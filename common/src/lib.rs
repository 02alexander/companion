#![no_std]

use core::usize;

use nalgebra::{Matrix3, SMatrix};
use serde::{Deserialize, Serialize};

pub const SAMPLE_TIME_MS: u32 = 10;

pub mod external {
    pub use nalgebra;
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LogMessage {
    pub time_ms: u64,
    pub control: f32,
    pub sensor_pend_angle: f32,
    pub sensor_wheel_velocity: f32,
    pub pend_angle: f32,
    pub pend_velocity: f32,
    pub wheel_velocity: f32,
}

type Mat<const R: usize, const C: usize> = SMatrix<f32, R, C>;


#[allow(non_snake_case)]
pub struct EKF<const NX: usize, const NY: usize> {
    pub state: Mat<NX, 1>,
    pub P: Mat<NX, NX>,
    pub A: Mat<NX, NX>,
    pub B: Mat<NX, 1>,
    pub C: Mat<NY, NX>,
    pub D: Mat<NY, 1>,
    pub R: Mat<NY, NY>,
    pub Q: Mat<NX, NX>,
}

impl Default for EKF<3, 3> {
    fn default() -> Self {
        EKF {
            state: Mat::zeros(),
            P: (0.01 as f32) * Mat::identity(),
            A: [
                [
                    0.9990661876105708,
                    -0.09282671358624058,
                    4.2937325870569804e-19,
                ],
                [
                    -0.07271524461036599,
                    1.002698728873369,
                    -3.3212237385430993e-18,
                ],
                [
                    0.0011928325090661347,
                    0.0013044989681259072,
                    0.9867551618071957,
                ],
            ]
            .into(),
            B: [[
                -0.009221120487808532,
                0.00042966099160485807,
                0.09940391188050599,
            ]]
            .into(),
            C: [
                [0.0, 0.0, 0.34260269786748027],
                [0.0, -2.4935789245789506, -0.01591774505100323],
                [
                    5.3297050155233245,
                    -0.001697272074671315,
                    0.03180841531133949,
                ],
            ]
            .into(),
            D: [[0.0, 0.0, 0.0]].into(),
            Q: [
                [
                    85195.8508266971,
                    2.2254670076019456e-06,
                    -0.0032695863581051893,
                ],
                [
                    2.2254670180918676e-06,
                    1.6309876860614014e-08,
                    -2.396194140971407e-05,
                ],
                [
                    -0.0032695863581098774,
                    -2.3961941398848557e-05,
                    0.035204133125493586,
                ],
            ]
            .into(),

            R: [
                [1.0, 0.0, 0.0],
                [0.0, 0.001, 0.0],
                [0.0, 0.0, 1.0],
            ]
            .into(),
        }
    }
}

impl<const NX: usize> EKF<NX, 3> {
    pub fn f(&self, x: Mat<NX, 1>) -> Mat<NX, 1> {
        self.A * x
    }

    pub fn fprim(&self, _x: Mat<NX, 1>) -> Mat<NX, NX> {
        self.A
    }

    pub fn h(&self, x: Mat<NX, 1>) -> Mat<3, 1> {
        let out = self.C * x;
        let ticks_vel = out[(0, 0)].abs();
        let pend_angle = out[(1, 0)];
        [ticks_vel, pend_angle, out[(2, 0)]].into()
    }

    pub fn hprim(&self, x: Mat<NX, 1>) -> Mat<3, NX> {
        let out = self.C * x;
        let ticks_vel = out[(0, 0)];
        let mut c = self.C;
        c[(0, 0)] *= ticks_vel.signum();
        c
    }

    pub fn time_update(&mut self, u: f32) {
        self.state = self.f(self.state) + self.B * u;
        self.P = self.Q + self.fprim(self.state) * self.P * self.fprim(self.state).transpose()
    }

    /// May fail if `S` is not invertible.
    pub fn measurment_update(&mut self, meas: Mat<3, 1>) -> Option<()> {
        let hprim = self.hprim(self.state);
        let s = self.R + hprim * self.P * hprim.transpose();
        let inv_s = s.try_inverse()?;
        let mut error = meas - self.h(self.state);

        error[0] = 0.0;
        error[2] = 0.0; // Ignore pendulum velocity estimate.

        let k = self.P * hprim.transpose() * inv_s;
        self.P = self.P - self.P * hprim.transpose() * inv_s * hprim * self.P;
        self.state = self.state + k * error;
        Some(())
    }
}
