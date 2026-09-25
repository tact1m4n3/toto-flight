const std = @import("std");

const hw = @import("hw.zig");
const time = @import("time.zig");
const parameter = @import("parameter.zig");
const math = @import("math.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const imu = @import("imu.zig");

// TODO: decide if we want to pass a timestamp with this attitude
pub var msg_attitude: Message(math.Quat) = .{};

pub const Params = extern struct {
    mah_kp: f32,
    mah_ki: f32,

    pub const default: Params = .{
        .mah_kp = 0.5,
        .mah_ki = 0.0,
    };
};

pub const AttitudeEstimator = struct {
    rcv_imu: Receiver(imu.Data) = undefined,

    ahrs: MahonyAhrs = .{},
    last_tick: ?time.Absolute = time.Absolute.from_us(0),

    pub fn init(estimator: *AttitudeEstimator, scheduler: *Scheduler) void {
        estimator.* = .{};

        imu.msg_data.subscribe(&estimator.rcv_imu, *AttitudeEstimator, estimator, imu_data_callback, scheduler);
    }

    fn imu_data_callback(estimator: *AttitudeEstimator, data: imu.Data) void {
        const now = hw.get_time_since_boot();
        const last_tick = estimator.last_tick orelse now;
        estimator.last_tick = now;
        const dt = now.diff(last_tick);
        const dt_f32 = dt.to_secs_f32();

        const params = parameter.get(struct {
            fuse: Params,
        });
        const cfg: MahonyAhrs.Config = .{
            .kp = params.fuse.mah_kp,
            .ki = params.fuse.mah_ki,
        };
        estimator.ahrs.update_imu(&cfg, data.gyro, data.accel, dt_f32);

        msg_attitude.publish(estimator.ahrs.quat);
    }
};

pub const MahonyAhrs = struct {
    quat: math.Quat = .identity,
    integral_error: math.Vec3 = .zero,

    pub const Config = struct {
        kp: f32,
        ki: f32,
    };

    /// gyro in rad/s, accel unit must be consistent
    pub fn update_imu(
        self: *MahonyAhrs,
        cfg: *const Config,
        gyro: math.Vec3,
        accel: math.Vec3,
        dt: f32,
    ) void {
        var error_total: math.Vec3 = .zero;

        const a_len_sq = accel.length_squared();
        if (a_len_sq > 1e-6) {
            const a = accel.div_scalar(std.math.sqrt(a_len_sq));

            // In NED, level resting accelerometer measures upward reaction force: [0, 0, -1].
            // Rotating NED [0, 0, -1] into body frame using conjugate(q):
            //   v_z = - (1 - 2*(x^2 + y^2))
            //   v_x = - (2*(x*z - w*y))
            //   v_y = - (2*(w*x + y*z))
            const q = self.quat;
            const v: math.Vec3 = .{
                .x = -2.0 * (q.x * q.z - q.w * q.y),
                .y = -2.0 * (q.w * q.x + q.y * q.z),
                .z = -(q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z),
            };

            // Error is the cross product between measured and estimated gravity direction
            error_total = a.cross(v);
        }

        self.integrate(cfg, gyro, error_total, dt);
    }

    /// gyro in rad/s, accel/mag unit must be consistent
    pub fn update(
        self: *MahonyAhrs,
        cfg: *const Config,
        gyro: math.Vec3,
        accel: math.Vec3,
        mag: math.Vec3,
        dt: f32,
    ) void {
        const q = self.quat;

        const m_len_sq = mag.length_squared();
        if (m_len_sq <= 1e-6) {
            self.update_imu(gyro, accel, dt);
            return;
        }
        // Magnetometer correction
        const m = mag.div_scalar(std.math.sqrt(m_len_sq));
        // Rotate measured magnetic vector into the estimated earth frame: h = q * m * q*
        const h = self.quat.rotate(m);
        // Project magnetic flux so Earth reference has zero dip in East (b_y = 0)
        // b = [sqrt(hx^2 + hy^2), 0, hz]
        const b_x = std.math.sqrt(h.x * h.x + h.y * h.y);
        const b_z = h.z;

        // Rotate reference Earth field back to body frame: w = q* * b * q
        const w: math.Vec3 = .{
            .x = 2.0 * b_x * (0.5 - q.y * q.y - q.z * q.z) + 2.0 * b_z * (q.x * q.z - q.w * q.y),
            .y = 2.0 * b_x * (q.x * q.y - q.w * q.z) + 2.0 * b_z * (q.w * q.x + q.y * q.z),
            .z = 2.0 * b_x * (q.w * q.y + q.x * q.z) + 2.0 * b_z * (0.5 - q.x * q.x - q.y * q.y),
        };

        var error_total: math.Vec3 = .zero;
        // Add magnetic heading error
        error_total = .add(error_total, m.cross(w));

        // Accelerometer correction
        const a_len_sq = accel.length_squared();
        if (a_len_sq > 1e-6) {
            const a = accel.div_scalar(std.math.sqrt(a_len_sq));
            const v: math.Vec3 = .{
                .x = -2.0 * (q.x * q.z - q.w * q.y),
                .y = -2.0 * (q.w * q.x + q.y * q.z),
                .z = -(q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z),
            };
            error_total = .add(error_total, a.cross(v));
        }

        self.integrate(cfg, gyro, error_total, dt);
    }

    fn integrate(self: *MahonyAhrs, cfg: *const Config, gyro: math.Vec3, err: math.Vec3, dt: f32) void {
        // Accumulate integral error
        if (cfg.ki > 0.0) {
            self.integral_error = .add(self.integral_error, err.mul_scalar(cfg.ki * dt));
        } else {
            self.integral_error = .zero;
        }

        // Apply PI feedback to gyro rates
        const corr_gyro = gyro.add(err.mul_scalar(cfg.kp)).add(self.integral_error);

        // First-order quaternion integration: q_dot = 0.5 * q * omega
        const q = self.quat;
        const half_dt = 0.5 * dt;

        const delta_q: math.Quat = .{
            .x = half_dt * (q.w * corr_gyro.x + q.y * corr_gyro.z - q.z * corr_gyro.y),
            .y = half_dt * (q.w * corr_gyro.y - q.x * corr_gyro.z + q.z * corr_gyro.x),
            .z = half_dt * (q.w * corr_gyro.z + q.x * corr_gyro.y - q.y * corr_gyro.x),
            .w = half_dt * (-q.x * corr_gyro.x - q.y * corr_gyro.y - q.z * corr_gyro.z),
        };
        self.quat = q.add(delta_q).normalize();
    }
};
