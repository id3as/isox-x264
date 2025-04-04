const std = @import("std");
const isox = @import("isox-common");
const stdio = @cImport({
    @cInclude("stdio.h");
});

const c = @cImport({
    @cInclude("stdint.h");
    @cInclude("x264.h");
});

const X264Error = error{
    FailToSetPresetTune,
    UnsupportedPixelFormat,
    BitrateMustBePositive,
    QPMustBePositive,
    RFMustBePositive,
    CreateFailed,
    FailedToSetProfile,
    EncoderClosed,
    UnsupportedPicStruct,
};

pub const std_options = .{
    .logFn = isox.log,
};

const Globals = struct {
    encoderResourceType: isox.ResourceType,
};

const EncoderResource = struct {
    env: isox.Env,
    resourceRef: isox.IsoxResourceRef,
    handle: ?*c.x264_t,
    params: c.x264_param_t,
    streamKey: isox.StreamKey,
};

const LogLevel = enum {
    LogNone,
    LogError,
    LogWarning,
    LogInfo,
    LogDebug,
};

const Preset = enum {
    UltraFast,
    SuperFast,
    VeryFast,
    Faster,
    Fast,
    Medium,
    Slow,
    Slower,
    VerySlow,
    Placebo,
};

const Encapsulation = enum {
    Mp4,
    Annexb,
};

const BitrateModeTag = enum {
    Abr,
    Cqp,
    Crf,
};

const BitrateMode = union(BitrateModeTag) {
    Abr: i64,
    Cqp: i64,
    Crf: f64,
};

const Tune = enum {
    Film,
    Animation,
    Grain,
    StillImage,
    Psnr,
    Ssim,
    FastDecode,
    ZeroLatency,
};

const NalHrd = enum {
    None,
    Vbr,
    Cbr,
};

const EncoderConfiguration = struct {
    logLevel: ?LogLevel,
    threads: ?i32,
    preset: Preset,
    constraints: i32,
    encapsulation: Encapsulation,
    profile: isox.H264Profile,
    level: i32,
    bitrateMode: BitrateMode,
    tune: ?Tune,
    keyFrameIntervalMin: ?i32,
    keyFrameIntervalMax: ?i32,
    bFrames: ?i32,
    frameReference: ?i32,
    cabac: ?bool,
    vbvMaxRate: ?i32,
    vbvBufferSize: ?i32,
    sceneCut: ?i32,
    aud: ?bool,
    noDeblock: ?bool,
    nalHrd: ?NalHrd,
    fullRecon: ?bool,
};

const CreateInstanceConfig = struct {
    encoderConfig: EncoderConfiguration,
    videoMetadata: isox.RawVideoMetadata,
    streamKey: isox.StreamKey,

    fn deinit(self: *const @This()) void {
        self.streamKey.deinit();
    }
};

const EncodeRequest = struct {
    frame: isox.RawVideoFrame,
    forceIdr: bool,

    pub fn deinit(self: *const @This(), allocator: std.mem.Allocator) void {
        self.frame.deinit(allocator);
    }
};

const UpdateInstanceRequestTag = enum {
    Encode,
    Flush,
};

const UpdateInstanceRequest = union(UpdateInstanceRequestTag) {
    Encode: EncodeRequest,
    Flush: isox.Unit,

    pub fn deinit(self: *const @This(), allocator: std.mem.Allocator) void {
        switch (self.*) {
            .Encode => |encode| encode.deinit(allocator),
            .Flush => {},
        }
    }
};

// fn x264_log_callback(_: ?*anyopaque, level: c_int, fmt: [*c]const u8, args: c.va_list) callconv(.C) void {
//     var buffer: [1024]u8 = undefined;
//     const len = stdio.vsnprintf(&buffer, buffer.len, fmt, args);

//     if (len < 0 or @as(usize, @intCast(len)) >= buffer.len) {
//         std.log.warn("x264_log_callback: Formatting error or output truncated.  fmt-string: {s}\n", .{fmt});
//         return;
//     }

//     switch (level) {
//         c.X264_LOG_ERROR => std.log.err("X264: {s}", .{buffer}),
//         c.X264_LOG_WARNING => std.log.warn("X264: {s}", .{buffer}),
//         c.X264_LOG_INFO => std.log.info("X264: {s}", .{buffer}),
//         c.X264_LOG_DEBUG => std.log.debug("X264: {s}", .{buffer}),
//         c.X264_LOG_NONE => {},
//         else => {},
//     }
// }

fn x264_param_default_preset(params: [*c]c.x264_param_t, preset: Preset, maybeTune: ?Tune) !void {
    const ret = c.x264_param_default_preset(
        params,
        switch (preset) {
            .UltraFast => "ultrafast",
            .SuperFast => "superfast",
            .VeryFast => "veryfast",
            .Faster => "faster",
            .Fast => "fast",
            .Medium => "medium",
            .Slow => "slow",
            .Slower => "slower",
            .VerySlow => "veryslow",
            .Placebo => "placebo",
        },
        blk: {
            if (maybeTune) |tune| {
                break :blk switch (tune) {
                    .Film => "film",
                    .Animation => "animation",
                    .Grain => "grain",
                    .StillImage => "stillImage",
                    .Psnr => "psnr",
                    .Ssim => "ssim",
                    .FastDecode => "fastDecode",
                    .ZeroLatency => "zeroLatency",
                };
            } else {
                break :blk null;
            }
        },
    );
    if (ret == 0) {
        return;
    } else {
        return X264Error.FailToSetPresetTune;
    }
}

fn x264_param_apply_profile(params: *c.x264_param_t, profile: isox.H264Profile) !void {
    const ret = c.x264_param_apply_profile(params, switch (profile) {
        isox.H264Profile.Baseline => "baseline",
        isox.H264Profile.Main => "main",
        isox.H264Profile.Extended => "extended",
        isox.H264Profile.High => "high",
        isox.H264Profile.High10 => "high10",
        isox.H264Profile.High422 => "high422",
        isox.H264Profile.High444 => "high444",
    });

    if (ret != 0) {
        return X264Error.FailedToSetProfile;
    }
}

fn x264_encoder_open(encoder: *const EncoderResource) !*c.x264_t {
    std.log.info("Opening encoder", .{});
    return c.x264_encoder_open_164(@constCast(&encoder.params)) orelse X264Error.CreateFailed; // The '164' is the x264 build...
}

fn x264_encoder_encode(handle: *c.x264_t, picIn: ?*c.x264_picture_t, picOut: *c.x264_picture_t) ![]c.x264_nal_t {
    var nals: [*c]c.x264_nal_t = undefined; // single pointer to array
    var numberOfNals: c_int = 0;

    const frameSize = c.x264_encoder_encode(handle, &nals, &numberOfNals, picIn, picOut);

    if (frameSize < 0) {
        std.log.err("x264_encoder_encode failed {}", .{frameSize});
        return &[_]c.x264_nal_t{};
    } else if (frameSize > 0) {
        return nals[0..@intCast(numberOfNals)];
    } else {
        return &[_]c.x264_nal_t{};
    }
}

fn btoi(value: ?bool) ?i32 {
    if (value) |val| {
        return @intFromBool(val);
    } else {
        return undefined;
    }
}

fn setIfPresent(field: anytype, value: ?@TypeOf(field.*)) void {
    field.* = value orelse field.*;
}

fn configureEncoder(env: isox.Env, encoder: *EncoderResource, encoderConfig: *const EncoderConfiguration, videoMetadata: *const isox.RawVideoMetadata) !void {
    encoder.params.i_timebase_num = 90000;
    encoder.params.i_timebase_den = 1;

    // My, isn't log-level wordy!
    encoder.params.i_log_level = if (std.process.getEnvVarOwned(env.allocator, "X264_LOG_LEVEL")) |value| blk: {
        defer env.allocator.free(value);
        if (std.mem.eql(u8, value, "none")) {
            break :blk c.X264_LOG_NONE;
        } else if (std.mem.eql(u8, value, "error")) {
            break :blk c.X264_LOG_ERROR;
        } else if (std.mem.eql(u8, value, "warning")) {
            break :blk c.X264_LOG_WARNING;
        } else if (std.mem.eql(u8, value, "info")) {
            break :blk c.X264_LOG_INFO;
        } else if (std.mem.eql(u8, value, "debug")) {
            break :blk c.X264_LOG_DEBUG;
        } else {
            break :blk c.X264_LOG_ERROR;
        }
    } else |_| blk: {
        if (encoderConfig.logLevel) |level| {
            break :blk switch (level) {
                .LogNone => c.X264_LOG_NONE,
                .LogError => c.X264_LOG_ERROR,
                .LogWarning => c.X264_LOG_WARNING,
                .LogInfo => c.X264_LOG_INFO,
                .LogDebug => c.X264_LOG_DEBUG,
            };
        } else {
            break :blk c.X264_LOG_ERROR;
        }
    };

    // Preset / tune
    try x264_param_default_preset(&encoder.params, encoderConfig.preset, encoderConfig.tune);

    // And then any overrides to the preset / tune
    setIfPresent(&encoder.params.i_threads, encoderConfig.threads);
    setIfPresent(&encoder.params.i_keyint_min, encoderConfig.keyFrameIntervalMin);
    setIfPresent(&encoder.params.i_keyint_max, encoderConfig.keyFrameIntervalMax);
    setIfPresent(&encoder.params.i_bframe, encoderConfig.bFrames);
    setIfPresent(&encoder.params.i_frame_reference, encoderConfig.frameReference);
    setIfPresent(&encoder.params.b_cabac, btoi(encoderConfig.cabac));
    setIfPresent(&encoder.params.rc.i_vbv_max_bitrate, encoderConfig.vbvMaxRate);
    setIfPresent(&encoder.params.rc.i_vbv_buffer_size, encoderConfig.vbvBufferSize);
    setIfPresent(&encoder.params.i_scenecut_threshold, encoderConfig.sceneCut);

    setIfPresent(&encoder.params.b_aud, btoi(encoderConfig.aud));
    setIfPresent(&encoder.params.b_deblocking_filter, btoi(encoderConfig.noDeblock));
    setIfPresent(&encoder.params.i_nal_hrd, if (encoderConfig.nalHrd) |hrd| switch (hrd) {
        .None => c.X264_NAL_HRD_NONE,
        .Vbr => c.X264_NAL_HRD_VBR,
        .Cbr => c.X264_NAL_HRD_CBR,
    } else null);
    setIfPresent(&encoder.params.b_full_recon, btoi(encoderConfig.fullRecon));
    encoder.params.b_annexb =
        switch (encoderConfig.encapsulation) {
        .Annexb => 1,
        .Mp4 => 0,
    };
    encoder.params.b_repeat_headers = 1;

    switch (encoderConfig.bitrateMode) {
        .Abr => |bitrate| {
            if (bitrate <= 0) {
                return X264Error.BitrateMustBePositive;
            }
            std.log.info("Bitrate is abr {}", .{bitrate});
            encoder.params.rc.i_bitrate = @intCast(bitrate);
        },
        .Cqp => |qp| {
            if (qp <= 0) {
                return X264Error.QPMustBePositive;
            }
            std.log.info("Bitrate is cqp {}", .{qp});
            encoder.params.rc.i_qp_constant = @intCast(qp);
        },
        .Crf => |rf| {
            if (rf <= 0) {
                return X264Error.RFMustBePositive;
            }
            std.log.info("Bitrate is crf {}", .{rf});
            encoder.params.rc.f_rf_constant = @floatCast(rf);
        },
    }

    encoder.params.i_width = @intCast(videoMetadata.resolution.width);
    encoder.params.i_height = @intCast(videoMetadata.resolution.height);

    encoder.params.vui.i_sar_width = @intCast(videoMetadata.pixelAspectRatio.fst);
    encoder.params.vui.i_sar_height = @intCast(videoMetadata.pixelAspectRatio.snd);

    switch (videoMetadata.frameRate) {
        .fixed => |fixed| {
            encoder.params.i_fps_num = @intCast(fixed.fst);
            encoder.params.i_fps_den = @intCast(fixed.snd);
        },
        .variable => {},
    }

    encoder.params.i_csp = switch (videoMetadata.pixelFormat) {
        isox.PixelFormat.Yuv420p => c.X264_CSP_I420,
        isox.PixelFormat.Yuv422p => c.X264_CSP_I422,
        isox.PixelFormat.Yuv444p => c.X264_CSP_I444,
        isox.PixelFormat.Yuv420p10le => c.X264_CSP_I420 | c.X264_CSP_HIGH_DEPTH,
        isox.PixelFormat.Yuv422p10le => c.X264_CSP_I422 | c.X264_CSP_HIGH_DEPTH,
        isox.PixelFormat.Yuv444p10le => c.X264_CSP_I444 | c.X264_CSP_HIGH_DEPTH,
        isox.PixelFormat.Nv12 => c.X264_CSP_NV12,
        isox.PixelFormat.Bgra => c.X264_CSP_BGRA,
        else => return X264Error.UnsupportedPixelFormat,
    };

    if (encoder.params.i_csp & c.X264_CSP_HIGH_DEPTH != 0) {
        encoder.params.i_bitdepth = 10;
    } else {
        encoder.params.i_bitdepth = 8;
    }

    encoder.params.i_level_idc = encoderConfig.level;
    try x264_param_apply_profile(&encoder.params, encoderConfig.profile);

    if (videoMetadata.colourInfo) |colourInfo| {
        encoder.params.vui.i_colorprim = @intCast(colourInfo.primaries);
        encoder.params.vui.i_transfer = @intCast(colourInfo.transferCharacteristics);
        encoder.params.vui.i_colmatrix = @intCast(colourInfo.matrixCoefficients);
        encoder.params.vui.i_chroma_loc = @intCast(colourInfo.chromaLocation);
        encoder.params.vui.b_fullrange = @intFromBool(colourInfo.fullRange);
    }

    encoder.params.b_stitchable = 0;
    encoder.params.b_vfr_input = 0;

    // encoder.params.pf_log = x264_log_callback;
    encoder.params.p_log_private = encoder;
}

fn maybeOutputFrame(env: isox.Env, encoder: *EncoderResource, picture: *c.x264_picture_t, nals: []c.x264_nal_t) !?isox.CompressedVideoFrame {
    if (nals.len == 0) {
        return null;
    }

    var nalSize: i64 = 0;
    for (nals) |nal| {
        nalSize += nal.i_payload;
    }

    // x264 guarantees this will work and that nals are sequential in memory
    const bin = try isox.IsoxBinary.new(env, @intCast(nalSize));
    @memcpy(bin.data, nals[0].p_payload[0..@intCast(nalSize)]);

    return isox.CompressedVideoFrame{
        .key = encoder.streamKey,
        .dts = .{
            .fst = picture.i_dts,
            .snd = 90000,
        },
        .pts = .{
            .fst = picture.i_pts,
            .snd = 90000,
        },
        .data = bin,
        .picStruct = switch (picture.i_pic_struct) {
            c.PIC_STRUCT_PROGRESSIVE => isox.PicStruct.progressive,
            c.PIC_STRUCT_TOP_BOTTOM => isox.PicStruct.tFF,
            c.PIC_STRUCT_BOTTOM_TOP => isox.PicStruct.bFF,
            else => return X264Error.UnsupportedPicStruct,
        },
        .frameType = switch (picture.i_type) {
            c.X264_TYPE_IDR => .HintIDR,
            else => .HintAny,
        },
        .duration = null,
        .metadata = null,
    };
}

fn encodeFrame(env: isox.Env, encoder: *EncoderResource, encodeRequest: *const EncodeRequest) !?isox.CompressedVideoFrame {
    if (encoder.handle) |handle| {
        var picIn: c.x264_picture_t = std.mem.zeroes(c.x264_picture_t);
        var picOut: c.x264_picture_t = std.mem.zeroes(c.x264_picture_t);

        c.x264_picture_init(&picIn);
        c.x264_picture_init(&picOut);

        picIn.img.i_csp = encoder.params.i_csp;
        picIn.i_pts = isox.ptsToTimestamp(encodeRequest.frame.pts, 90000);

        if (encodeRequest.forceIdr) {
            picIn.i_type = c.X264_TYPE_IDR;
        }

        switch (encodeRequest.frame.frameData.picStruct) {
            .progressive => picIn.i_pic_struct = c.PIC_STRUCT_PROGRESSIVE,
            .tFF => picIn.i_pic_struct = c.PIC_STRUCT_TOP_BOTTOM,
            .bFF => picIn.i_pic_struct = c.PIC_STRUCT_BOTTOM_TOP,
        }

        for (encodeRequest.frame.frameData.planes, 0..) |plane, i| {
            picIn.img.plane[i] = plane.binary.data.ptr;
            picIn.img.i_stride[i] = @intCast(plane.stride);
        }

        const nals = try x264_encoder_encode(handle, &picIn, &picOut);

        return maybeOutputFrame(env, encoder, &picOut, nals);
    } else {
        return X264Error.EncoderClosed;
    }
}

fn flushEncoder(env: isox.Env, allocator: std.mem.Allocator, encoder: *EncoderResource) ![]isox.CompressedVideoFrame {
    if (encoder.handle) |handle| {
        var frames = std.ArrayList(isox.CompressedVideoFrame).init(allocator);
        defer frames.deinit();

        const remainingFrames: c_int = c.x264_encoder_delayed_frames(handle);

        std.log.info("Flush: there are {} remaining frames", .{remainingFrames});

        while (c.x264_encoder_delayed_frames(handle) > 0) {
            var picOut: c.x264_picture_t = std.mem.zeroes(c.x264_picture_t);
            c.x264_picture_init(&picOut);

            const nals = try x264_encoder_encode(handle, null, &picOut);

            if (try maybeOutputFrame(env, encoder, &picOut, nals)) |frame| {
                try frames.append(frame);
            }

            if (nals.len == 0) {
                std.time.sleep(1); // We have at least one frame left, but didn't get output - frame must still be processing, sleep and retry...
            }
        }
        return frames.toOwnedSlice();
    } else {
        return X264Error.EncoderClosed;
    }
}

fn load(env: isox.Env, _: isox.Term) !*anyopaque {
    const globals = try std.heap.c_allocator.create(Globals);
    globals.encoderResourceType = try isox.createResourceType(env, encoderResourceDestructor);
    return globals;
}

fn query(_: isox.Env, _: isox.Term) !isox.QueryReturn {
    return null;
}

fn queryInstance(_: isox.Env, _: isox.IsoxResource, _: isox.Term) !isox.QueryInstanceReturn {
    return null;
}

fn createInstance(env: isox.Env, args: isox.Term) !isox.CreateInstanceReturn {
    const globals: *Globals = @alignCast(@ptrCast(env.privData()));
    const config = try isox.decode(CreateInstanceConfig, env, args);

    defer config.deinit();

    std.log.info("x264 ISOX CreateInstance {?}", .{config});

    const encoder: *EncoderResource = try isox.allocResourceData(env, EncoderResource);
    errdefer isox.freeResourceData(encoder);

    encoder.env = isox.allocEnv();
    encoder.streamKey = try config.streamKey.copy(encoder.env);

    try configureEncoder(env, encoder, &config.encoderConfig, &config.videoMetadata);

    encoder.handle = try x264_encoder_open(encoder);

    const encoderTerm = try isox.IsoxResource.new(env, globals.encoderResourceType, encoder);

    return .{ encoderTerm, null };
}

fn updateInstance(env: isox.Env, resource: isox.IsoxResource, args: isox.Term) !isox.UpdateInstanceReturn {
    const request = try isox.decode(UpdateInstanceRequest, env, args);
    defer request.deinit(env.allocator);

    const encoder: *EncoderResource = @ptrCast(@alignCast(resource.obj));

    const allocator = std.heap.c_allocator;

    switch (request) {
        .Encode => |encodeRequest| {
            const result = try encodeFrame(env, encoder, &encodeRequest);
            const frames = if (result) |frame| &[_]isox.CompressedVideoFrame{frame} else &[_]isox.CompressedVideoFrame{};
            return isox.encode([]const isox.CompressedVideoFrame, env, frames);
        },
        .Flush => {
            const frames = try flushEncoder(env, allocator, encoder);
            defer allocator.free(frames);
            return isox.encode([]const isox.CompressedVideoFrame, env, frames);
        },
    }
}

fn destroyInstance(_: isox.Env, resource: isox.IsoxResource) !isox.DestroyInstanceReturn {
    const encoder: *EncoderResource = @ptrCast(@alignCast(resource.obj));

    if (encoder.handle) |handle| {
        c.x264_encoder_close(handle);
        encoder.handle = null;
    }

    return;
}

fn unload() void {}

////////////////////////////////////////////////////////////////////////////////////////////
// Destructor functions
fn encoderResourceDestructor(_: isox.Env, obj: *anyopaque) void {
    const encoder: *EncoderResource = @alignCast(@ptrCast(obj));

    if (encoder.handle) |handle| {
        c.x264_encoder_close(handle);
    }
    encoder.streamKey.deinit();

    isox.freeResourceData(encoder);
}

export fn isox_init() *const isox.IsoxExtension {
    return isox.init(&isox.IsoxExtensionConfig{
        .name = "x264",
        .load = load,
        .query = query,
        .createInstance = createInstance,
        .queryInstance = queryInstance,
        .updateInstance = updateInstance,
        .destroyInstance = destroyInstance,
        .unload = unload,
    });
}

test "x264-createInstance" {
    const env = isox.allocEnv();
    env.setPrivData(try load(env, try isox.encode(i64, env, 1)));

    const streamKey = isox.StreamKey{
        .sourceName = isox.IsoxString.newLiteral("testSource"),
        .programNumber = 1,
        .streamId = 1,
        .renditionName = isox.IsoxString.newLiteral("default"),
    };

    // Create the encoder...
    const config = CreateInstanceConfig{
        .streamKey = streamKey,
        .encoderConfig = EncoderConfiguration{
            .logLevel = LogLevel.LogDebug,
            .threads = null,
            .preset = .UltraFast,
            .constraints = 0,
            .encapsulation = .Mp4,
            .profile = .Main,
            .level = 40,
            .bitrateMode = BitrateMode{ .Abr = 5000 },
            .tune = null,
            .keyFrameIntervalMin = null,
            .keyFrameIntervalMax = null,
            .bFrames = null,
            .frameReference = null,
            .cabac = null,
            .vbvMaxRate = null,
            .vbvBufferSize = null,
            .sceneCut = null,
            .aud = null,
            .noDeblock = null,
            .nalHrd = null,
            .fullRecon = null,
        },
        .videoMetadata = isox.RawVideoMetadata{
            .pixelFormat = isox.PixelFormat.Yuv420p,
            .resolution = isox.Resolution{ .width = 1920, .height = 1080 },
            .frameRate = isox.FrameRate{ .fixed = .{ .fst = 25, .snd = 1 } },
            .pixelAspectRatio = isox.PixelAspectRatio{ .fst = 1, .snd = 1 },
            .colourInfo = null,
        },
    };

    const configTerm = try isox.encode(CreateInstanceConfig, env, config);
    const result = try createInstance(env, configTerm);
    const encoder = result[0];

    // And encode a frame...
    const encodeRequest = UpdateInstanceRequest{
        .Encode = EncodeRequest{
            .forceIdr = false,
            .frame = isox.RawVideoFrame{
                .key = streamKey,
                .pts = isox.Pts{ .fst = 0, .snd = 1 },
                .duration = isox.Duration{ .fst = 40, .snd = 1000 },
                .frameData = isox.VideoFrameData{
                    .planes = &[_]isox.VideoPlane{
                        isox.VideoPlane{
                            .stride = 1920,
                            .binary = try isox.IsoxBinary.new(env, 1920 * 1080),
                        },
                        isox.VideoPlane{
                            .stride = 1920 / 2,
                            .binary = try isox.IsoxBinary.new(env, 1920 * 1080 / 4),
                        },
                        isox.VideoPlane{
                            .stride = 1920 / 2,
                            .binary = try isox.IsoxBinary.new(env, 1920 * 1080 / 4),
                        },
                    },
                    .bufferHeight = 1080,
                    .picStruct = isox.PicStruct.progressive,
                    .originalMetadata = null,
                },
            },
        },
    };
    const encodeTerm = try isox.encode(UpdateInstanceRequest, env, encodeRequest);

    const encodeResult = try updateInstance(env, encoder, encodeTerm) orelse @panic("null returned from updateInstance");
    const encodeFrames = try isox.decode([]isox.CompressedVideoFrame, env, encodeResult);

    // And flush
    const flushRequest = UpdateInstanceRequest{ .Flush = isox.Unit.isoxUnit };
    const flushTerm = try isox.encode(UpdateInstanceRequest, env, flushRequest);

    const flushResult = try updateInstance(env, encoder, flushTerm) orelse @panic("null returned from updateInstance");
    const flushFrames = try isox.decode([]isox.CompressedVideoFrame, env, flushResult);

    // And see what we got returned
    try std.testing.expect(encodeFrames.len == 0);
    try std.testing.expect(flushFrames.len == 1);

    // try std.testing.expectEqualStrings(flushFrames[0].typeName, @typeName(isox.ExtensionMessage));
    // const m: *const isox.ExtensionMessage = @alignCast(@ptrCast(messages[0].ptr));

    // try switch (m.message) {
    //     .CompressedVideoFrame => |_| {},
    //     else => error.UnexpectedMessageType,
    // };

    for (encodeFrames) |frame| {
        frame.deinit();
    }
    env.allocator.free(encodeFrames);
    for (flushFrames) |frame| {
        frame.deinit();
    }
    env.allocator.free(flushFrames);

    // Finally, clean up
    _ = try destroyInstance(env, encoder);

    // Cleanup - would be automatic outside of tests
    encoderResourceDestructor(env, encoder.obj);

    isox.freeEnv(env);
}
