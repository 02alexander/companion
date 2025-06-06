#![no_std]

use core::usize;

use nalgebra::{Matrix3, SMatrix};
use serde::{Deserialize, Serialize};

const SAMPLE_TIME_MS: u32 = 50;

pub mod external {
    pub use nalgebra;
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LogMessage {
    pub time_ms: u64,
    pub control: f32,
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
    pub R: Mat<NY, NY>,
    pub Q: Mat<NX, NX>,
}

impl Default for EKF<3,2> {
    fn default() -> Self {
        EKF {
            state: Mat::zeros(),
            P: (10.0 as f32)*Matrix3::identity(),
            A: [
                [0.93550699, 0.0, 0.0],
                [0.0, 1.0, SAMPLE_TIME_MS as f32 / 1000.0],
                [0.0, 0.0, 1.0],
            ].into(),
            B: [33.22814019, 0.0, 0.0].into(),
            C: [
                [1.0, 0.0],
                [0.0, 1.0],
                [0.0, 0.0],
            ].into(),
            R: [
                [0.1, 0.0],
                [0.0, 0.0001],
            ].into(),
            Q: 0.1 as f32*Mat::identity(),
        }
    }
}

impl<const NX: usize> EKF<NX, 2> {

    pub fn f(&self, x: Mat<NX,1>) -> Mat<NX,1> {
        self.A * x
    }

    pub fn fprim(&self, _x: Mat<NX,1>) -> Mat<NX,NX> {
        self.A
    }

    pub fn h(&self, x: Mat<NX,1>) -> Mat<2,1> {
        let out = self.C * x;
        let ticks_vel = out[(0,0)].abs();
        let pend_angle = out[(1,0)];
        [ticks_vel, pend_angle].into()
    }

    pub fn hprim(&self, x: Mat<NX,1>) -> Mat<2,NX> {
        let out = self.C * x;
        let ticks_vel = out[(0,0)];
        // let _pend_angle = out[(1,0)];
        let mut out = self.C;
        out[(0,0)] *= ticks_vel.signum();
        out
    }

    pub fn time_update(&mut self, u: f32,) {
        self.state = self.f(self.state) + self.B*u;
        self.P = self.Q + self.fprim(self.state) * self.P * self.fprim(self.state).transpose()
    }

    /// May fail if `S` is not invertible.
    pub fn measurment_update(&mut self, meas: Mat<2,1>) -> Option<()> {
        let hprim = self.hprim(self.state);
        let s = self.R + hprim * self.P * hprim.transpose();
        let inv_s = s.try_inverse()?;
        let error = meas - self.h(self.state);

        let k = self.P * hprim.transpose() * inv_s;
        self.P = self.P - self.P * hprim.transpose() * inv_s * hprim * self.P;
        self.state = self.state + k * error;
        Some(())
    }
}


