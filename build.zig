const std = @import("std");

const testTargets = [_]std.Target.Query{
    .{}, // native
};

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(.{ .cpu_model = if (b.graph.host.result.cpu.arch.isX86()) .{ .explicit = &std.Target.x86.cpu.haswell } else .native });
    const optimize = b.standardOptimizeOption(.{});

    // ISOX tests
    const test_step = b.step("test", "Run unit tests");

    build_x264(b, &target, optimize, test_step);
}

fn build_x264(b: *std.Build, target: *const std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode, test_step: *std.Build.Step) void { // load the "zig-speak" dependency from build.zig.zon
    const isox_common = b.dependency("isox-common", .{
        .optimize = optimize,
    });
    const isox_module = isox_common.module("isox-common");

    var x264 = b.addSharedLibrary(.{
        .name = "x264",
        .root_source_file = .{ .cwd_relative = "src/x264.zig" },
        .target = target.*,
        .optimize = optimize,
        .version = .{
            .major = 0,
            .minor = 1,
            .patch = 0,
        },
    });
    x264.root_module.addImport("isox-common", isox_module);
    x264.addIncludePath(.{ .cwd_relative = "../isox_host/c_src" });
    x264.linkSystemLibrary("x264");
    x264.linkLibC();
    if (b.graph.host.result.os.tag == std.Target.Os.Tag.macos) {
        x264.linker_allow_shlib_undefined = true;
    }
    const x264_lib = b.addInstallArtifact(x264, .{ .dest_dir = .{ .override = .{ .custom = "../../priv/isox" } } });
    b.getInstallStep().dependOn(&x264_lib.step);

    const unit_tests = b.addTest(.{
        .root_source_file = b.path("src/x264.zig"),
        .target = target.*,
        .link_libc = true,
    });
    unit_tests.linkSystemLibrary("x264");
    unit_tests.linkSystemLibrary("isox_host");
    unit_tests.addIncludePath(.{ .cwd_relative = "../c_src" });
    unit_tests.addLibraryPath(.{ .cwd_relative = "../priv/isox" });

    const run_unit_tests = b.addRunArtifact(unit_tests);
    run_unit_tests.has_side_effects = true;
    run_unit_tests.step.name = "X264 Tests";
    test_step.dependOn(&run_unit_tests.step);
}
