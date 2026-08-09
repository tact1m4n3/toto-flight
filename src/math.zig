pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub const zero: Vec3 = .{ .x = 0.0, .y = 0.0, .z = 0.0 };
    pub const one: Vec3 = .{ .x = 1.0, .y = 1.0, .z = 1.0 };

    pub fn add(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.x + b.x,
            .y = a.y + b.y,
            .z = a.z + b.z,
        };
    }

    pub fn sub(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.x - b.x,
            .y = a.y - b.y,
            .z = a.z - b.z,
        };
    }

    pub fn scale(v: Vec3, s: f32) Vec3 {
        return .{
            .x = v.x * s,
            .y = v.y * s,
            .z = v.z * s,
        };
    }

    pub fn component_mul(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.x * b.x,
            .y = a.y * b.y,
            .z = a.z * b.z,
        };
    }

    pub fn component_div(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.x / b.x,
            .y = a.y / b.y,
            .z = a.z / b.z,
        };
    }

    pub fn transform(v: Vec3, mat: Mat3) Vec3 {
        return .{
            .x = mat.m[0][0] * v.x + mat.m[0][1] * v.y + mat.m[0][2] * v.z,
            .y = mat.m[1][0] * v.x + mat.m[1][1] * v.y + mat.m[1][2] * v.z,
            .z = mat.m[2][0] * v.x + mat.m[2][1] * v.y + mat.m[2][2] * v.z,
        };
    }
};

pub const Mat3 = struct {
    m: [3][3]f32,

    pub const identity: Mat3 = .{
        .m = .{
            .{ 1.0, 0.0, 0.0 },
            .{ 0.0, 1.0, 0.0 },
            .{ 0.0, 0.0, 1.0 },
        },
    };
};
