use core::f32::consts::PI;

use nalgebra::SMatrix;

use crate::model::{INERTIA_RATIO, WHEEL_STATIC_GAIN, WHEEL_TIME_CONSTANT};

type Mat<const R: usize, const C: usize> = SMatrix<f32, R, C>;

pub const NU: usize = 11;

pub fn cont(min: f32, max: f32, n: usize, k: usize) -> f32 {
    let interval_size = (max - min) / (n as f32 - 1.0);
    interval_size * k as f32 + min
}

pub fn cont_u(du: usize) -> f32 {
    cont(-1.0, 1.0, NU, du)
}

#[derive(Clone, Copy, Debug)]
pub struct NLPendulumModel {
    pub omegaf: f32,
    pub theta: f32,
    pub thetadot: f32,
}
pub const DT: f32 = 0.05;

impl NLPendulumModel {
    pub fn bottom() -> NLPendulumModel {
        NLPendulumModel {
            omegaf: 0.0,
            theta: PI,
            thetadot: 0.0,
        }
    }

    pub fn step(&mut self, u: f32, dt: f32) {
        let g = 9.81;
        let r = 0.145;
        let domegaf = (WHEEL_STATIC_GAIN * u - self.omegaf) / WHEEL_TIME_CONSTANT;
        let dtheta = self.thetadot;
        let dthetadot =
            -INERTIA_RATIO * 2.0 * domegaf - 0.2 * self.thetadot + g / r * libm::sinf(self.theta);
        self.omegaf += dt * domegaf;
        self.theta += dt * dtheta;
        self.thetadot += dt * dthetadot;

        if self.theta > PI {
            self.theta -= 2.0 * PI;
        } else if self.theta < -PI {
            self.theta += 2.0 * PI;
        }
    }

    pub fn from_vec(v: Mat<3, 1>) -> NLPendulumModel {
        NLPendulumModel {
            omegaf: v[0],
            theta: v[1],
            thetadot: v[2],
        }
    }

    pub fn as_vec(&self) -> Mat<3, 1> {
        [self.omegaf, self.theta, self.thetadot].into()
    }
}

pub const NPARAM: usize = 5;

pub struct Agent {
    pub parameters: Mat<NU, NPARAM>,
}

pub fn powi(x: f32, n: u8) -> f32 {
    let mut res = 1.0;
    for _ in 0..n {
        res = res * x
    }
    res
}

impl Agent {
    pub fn new() -> Agent {
        Agent {
            parameters: Mat::zeros(),
        }
    }

    pub fn h_with_params(state: &Mat<3, 1>, params: Mat<1, NPARAM>) -> f32 {
        params[0] * state[1]
            + params[1] * powi(state[1], 2)
            + params[2] * state[2]
            + params[3] * powi(state[2], 2)
            + params[4]
    }

    pub fn grad_h(state: &Mat<3,1>, params: Mat<1, NPARAM>) -> Mat<1, NPARAM> {
        [
            state[1],
            powi(state[1], 2),
            state[2],
            powi(state[2], 2),
            1.0,
        ].into()
    }

    pub fn eval_all_actions(&self, state: &Mat<3,1>) -> Mat<NU, 1> {
        let mut exps = [0.0_f32; NU];
        for action in 0..NU {
            exps[action] = libm::expf(Self::h_with_params(state, self.parameters.row(action).into()));
        }
        let sum: f32 = exps.iter().sum();
        for action in 0..NU {
            exps[action] /= sum;
        }
        exps.into()
    }

    pub fn eval(&self, state: &Mat<3, 1>, action: usize) -> f32 {
        let mut exps = [0.0_f32; NU];
        for i in 0..NU {
            exps[i] = libm::expf(Self::h_with_params(state, self.parameters.row(action).into()));
        }
        exps[action]/exps.iter().sum::<f32>()
    }

    pub fn grad(&self, state: &Mat<3, 1>, action: usize) -> Mat<NU, NPARAM> {
        unimplemented!()
    }

    pub fn action_value(&self, state: &Mat<3, 1>, action: usize) -> f32 {
        let mut state = NLPendulumModel::from_vec(state.clone());
        let u = cont_u(action);
        for _ in 0..5 {
            state.step(u as f32, DT / 5.0);
        }
        self.eval(&state.as_vec(), action)
    }

    pub fn best_action(&self, state: &Mat<3, 1>) -> usize {
        (0..NU)
            .max_by(|&a1, &a2| {
                self.action_value(&state, a1)
                    .partial_cmp(&self.action_value(&state, a2))
                    .unwrap()
            })
            .unwrap()
    }
}
