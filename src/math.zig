const std = @import("std");
const math = std.math;

pub const degrees_from_radians = math.radiansToDegrees;
pub const radians_from_degrees = math.degreesToRadians;

pub const Axis = enum {
    x,
    y,
    z,
};

pub const Vec3 = extern struct {
    x: f32,
    y: f32,
    z: f32,

    pub const zero: Vec3 = .{ .x = 0.0, .y = 0.0, .z = 0.0 };
    pub const one: Vec3 = .{ .x = 1.0, .y = 1.0, .z = 1.0 };

    pub fn axis(comptime a: Axis) Vec3 {
        switch (a) {
            .x => return .{ .x = 1.0, .y = 0.0, .z = 0.0 },
            .y => return .{ .x = 0.0, .y = 1.0, .z = 0.0 },
            .z => return .{ .x = 0.0, .y = 0.0, .z = 1.0 },
        }
    }

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

    pub fn mul(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.x * b.x,
            .y = a.y * b.y,
            .z = a.z * b.z,
        };
    }

    pub fn div(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.x / b.x,
            .y = a.y / b.y,
            .z = a.z / b.z,
        };
    }

    pub fn mul_scalar(v: Vec3, s: f32) Vec3 {
        return .{
            .x = v.x * s,
            .y = v.y * s,
            .z = v.z * s,
        };
    }

    pub fn div_scalar(v: Vec3, s: f32) Vec3 {
        return .{
            .x = v.x / s,
            .y = v.y / s,
            .z = v.z / s,
        };
    }

    pub fn clamp(v: Vec3, min: f32, max: f32) Vec3 {
        return .{
            .x = std.math.clamp(v.x, min, max),
            .y = std.math.clamp(v.y, min, max),
            .z = std.math.clamp(v.z, min, max),
        };
    }

    pub fn negate(v: Vec3) Vec3 {
        return .{ .x = -v.x, .y = -v.y, .z = -v.z };
    }

    pub fn length_squared(v: Vec3) f32 {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    pub fn length(v: Vec3) f32 {
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    pub fn normalize(v: Vec3) Vec3 {
        return v.div_scalar(v.length());
    }

    pub fn dot(a: Vec3, b: Vec3) f32 {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    pub fn cross(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.y * b.z - a.z * b.y,
            .y = a.z * b.x - a.x * b.z,
            .z = a.x * b.y - a.y * b.x,
        };
    }

    /// Rotates vector `v` around a normalized `axis` by `angle_radians`.
    pub fn rotate_around_axis(v: Vec3, axis_vec: Vec3, angle_radians: f32) Vec3 {
        const c = math.cos(angle_radians);
        const s = math.sin(angle_radians);
        // Rodrigues' rotation formula:
        // v_rot = v * cos + (axis x v) * sin + axis * (axis . v) * (1 - cos)
        const term1 = v.mul_scalar(c);
        const term2 = axis_vec.cross(v).mul_scalar(s);
        const term3 = axis_vec.mul_scalar(axis_vec.dot(v) * (1.0 - c));
        return term1.add(term2).add(term3);
    }
};

pub const Mat3 = extern struct {
    m: [3][3]f32,

    pub const identity: Mat3 = .{
        .m = .{
            .{ 1.0, 0.0, 0.0 },
            .{ 0.0, 1.0, 0.0 },
            .{ 0.0, 0.0, 1.0 },
        },
    };

    pub fn init_from_rows(row0: Vec3, row1: Vec3, row2: Vec3) Mat3 {
        return .{
            .m = .{
                .{ row0.x, row0.y, row0.z },
                .{ row1.x, row1.y, row1.z },
                .{ row2.x, row2.y, row2.z },
            },
        };
    }

    pub fn transpose(mat: Mat3) Mat3 {
        return .{
            .m = .{
                .{ mat.m[0][0], mat.m[1][0], mat.m[2][0] },
                .{ mat.m[0][1], mat.m[1][1], mat.m[2][1] },
                .{ mat.m[0][2], mat.m[1][2], mat.m[2][2] },
            },
        };
    }

    pub fn transform(mat: *const Mat3, v: Vec3) Vec3 {
        return .{
            .x = mat.m[0][0] * v.x + mat.m[0][1] * v.y + mat.m[0][2] * v.z,
            .y = mat.m[1][0] * v.x + mat.m[1][1] * v.y + mat.m[1][2] * v.z,
            .z = mat.m[2][0] * v.x + mat.m[2][1] * v.y + mat.m[2][2] * v.z,
        };
    }
};

pub const Quat = extern struct {
    x: f32,
    y: f32,
    z: f32,
    w: f32,

    pub const identity: Quat = .{ .x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0 };

    /// Asserts the axis vector is normalized.
    pub fn from_axis_angle(axis: Vec3, angle_radians: f32) Quat {
        const half_angle = angle_radians / 2.0;
        const s = math.sin(half_angle);
        return .{
            .x = axis.x * s,
            .y = axis.y * s,
            .z = axis.z * s,
            .w = math.cos(half_angle),
        };
    }

    pub fn to_euler_angles(q: Quat) Vec3 {
        const sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        const cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        const roll = math.atan2(sinr_cosp, cosr_cosp);

        const sinp = 2.0 * (q.w * q.y - q.x * q.z);
        const pitch = if (@abs(sinp) >= 1.0)
            math.copysign(@as(f32, math.pi / 2.0), sinp)
        else
            math.asin(sinp);

        const siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const yaw = math.atan2(siny_cosp, cosy_cosp);

        return .{ .x = roll, .y = pitch, .z = yaw };
    }

    pub fn add(a: Quat, b: Quat) Quat {
        return .{
            .x = a.x + b.x,
            .y = a.y + b.y,
            .z = a.z + b.z,
            .w = a.w + b.w,
        };
    }

    pub fn mul(a: Quat, b: Quat) Quat {
        return .{
            .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        };
    }

    pub fn length_squared(q: Quat) f32 {
        return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    }

    pub fn length(q: Quat) f32 {
        return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    }

    pub fn conjugate(q: Quat) Quat {
        return .{ .x = -q.x, .y = -q.y, .z = -q.z, .w = q.w };
    }

    // Asserts q is normalized.
    pub fn rotate(q: Quat, v: Vec3) Vec3 {
        const q_vec: Vec3 = .{ .x = q.x, .y = q.y, .z = q.z };
        const t = q_vec.cross(v).mul_scalar(2.0);
        return v.add(t.mul_scalar(q.w)).add(q_vec.cross(t));
    }

    pub fn normalize(q: Quat) Quat {
        const len = q.length();
        return .{ .x = q.x / len, .y = q.y / len, .z = q.z / len, .w = q.w / len };
    }
};
