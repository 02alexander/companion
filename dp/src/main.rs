use std::{f64::{consts::PI, INFINITY}, fs::OpenOptions, io::Write, os::linux::raw::stat};

use common::dp::{cont, disc_state};
use nalgebra::SMatrix;
use ordered_float::NotNan;
use rand::{rngs::SmallRng, Rng, SeedableRng};

pub type Mat<const R: usize, const C: usize> = SMatrix<f64, R, C>;

#[derive(Clone, Copy, Debug)]
struct PendulumState {
    omegaf: f64,
    theta: f64,
    thetadot: f64,
}

impl PendulumState {

    pub fn bottom() -> PendulumState {
        PendulumState { omegaf:  0.0, theta: PI, thetadot: 0.0 }
    }

    pub fn step(&mut self, u: f64, dt: f64) {
        let ir = 0.03320426557380722 * 2.0;
        let T = 0.7;
        let g = 9.81;
        let r = 0.145;
        let domegaf = dt * (330.0 * u - self.omegaf) / T;
        let dtheta = dt * self.thetadot;
        let dthetadot = dt
            * (-ir * (330.0 * u - self.omegaf) / T - 0.2 * self.thetadot
                + g / r * self.theta.sin());

        self.omegaf += domegaf;
        self.theta += dtheta;
        self.thetadot += dthetadot;

        if self.theta > PI {
            self.theta -= 2.0 * PI;
        } else if self.theta < -PI {
            self.theta += 2.0 * PI;
        }

    }

    pub fn as_canon_vec(&self) -> Mat<3,1> {
        if self.theta < 0.0 {
            -self.as_vec()
        } else {
            self.as_vec()
        }
    }

    pub fn as_vec(&self) -> Mat<3,1> {
        [
            self.omegaf/100.0,
            self.theta,
            self.thetadot/10.0,
        ].into()
    }

}

const NU: usize = 11;
const NPARAM: usize = 4;

fn cont_u(du: usize) -> f32 {
    cont(-1.0, 1.0, NU, du)
}

struct Agent {
    parameters: Mat<NU,NPARAM>,
}

fn append1(m: Mat<3,1>) -> Mat<4,1> {
    [m[0], m[1], m[2], 1.0].into()
}

impl Agent {
    pub fn new() -> Agent {
        Agent {
            parameters: Mat::zeros(),
        }
    }
    pub fn eval(&self , state: &PendulumState, action: usize) -> f64 {
        (self.parameters.row(action) * append1(state.as_canon_vec()))[0]
    }

    pub fn grad(&self, state: &PendulumState, action: usize) -> Mat<NU,NPARAM> {
        let mut zeros: Mat<NU,NPARAM> = Mat::zeros();

        zeros.set_row(action, &append1(state.as_canon_vec()).transpose());
        zeros
    }

    pub fn best_action(&self, state: &PendulumState) -> usize {
        (0..NU).max_by_key(|&action| NotNan::new(self.eval(state, action)).unwrap()).unwrap()
    }

    pub fn max_qsa(&self, state: &PendulumState) -> f64 {
        (0..NU).map(|action| NotNan::new(self.eval(state, action)).unwrap()).max().unwrap().into()
    }
}

fn simulate_episode(agent: &Agent, epsilon: f64, n: usize) -> Vec<(PendulumState, usize, f64)> {
    let mut trajectory = Vec::new();
    let mut rng = SmallRng::from_os_rng();
    let mut state = PendulumState::bottom();
    for _ in 0..n {
        let action = if rng.random_bool(epsilon) {
            rng.random_range(0..NU)
        } else {
            agent.best_action(&state)
        };

        let u = cont_u(action);
        let cur_state = state.clone();
        for _ in 0..10 {
            state.step(u as f64, 0.03/10.0);
        }

        trajectory.push((cur_state, action, reward(&state)));
    }
    trajectory
}

fn reward(state: &PendulumState) -> f64 {
    assert!(state.theta.abs() < 4.0);
    (4.0 - state.theta.abs()).powi(2)
}


fn main() {
    let mut agent = Agent::new();

    let learning_rate = 0.000000001;
    let discount = 0.995;
    
    let mut latest_trajectory = Vec::new();
    for _ in 0..10000 {
        
        let n = 1000;
        let trajectory = simulate_episode(&agent, 0.15, n);
        let mut tot_reward = 0.0;
        // for i in 0..1 {

        let mut eligibility_trace = agent.grad(&trajectory[0].0, 0);
        eligibility_trace = Mat::zeros();
        let lambda = 0.9;

        for i in (0..trajectory.len()-1).rev() {
            let (state, action, reward) = trajectory[i].clone();
            tot_reward += reward;
            let next_state = trajectory[i+1].0;
            
            let qsa = agent.eval(&state, action);
            let target = reward + discount*agent.max_qsa(&next_state);
            // let target = reward + discount*agent.(&next_state);

            let error = qsa - target;
            let grad = agent.grad(&state, action);

            eligibility_trace = eligibility_trace*lambda + grad;

            agent.parameters += eligibility_trace*error*learning_rate;
        }
        println!("{:.5}", tot_reward/n as f64);
        latest_trajectory = trajectory;
    }


    for (state, action, reward) in simulate_episode(&agent, 0.0, 500) {
        println!("{:.5} {} {:.5}", state.theta, action, cont_u(action));
    }
    println!("{}", agent.parameters);
}