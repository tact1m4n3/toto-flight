const std = @import("std");
const c = @import("jsbsim_c");

pub fn main(init: std.process.Init) !void {
    const fdmex = c.jsbsim_create("assets");
    defer c.jsbsim_destroy(fdmex);

    std.debug.assert(c.jsbsim_load_model(fdmex, "smallrcwing"));

    try std.Io.File.stdout().writeStreamingAll(init.io, "Hello world!");
}

pub const JSBSim_Error = error{JSBSim_Error};
