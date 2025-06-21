use std::{
    f32::consts::PI,
};

use common::dp::{cont, cont_u, Agent, NLPendulumModel, DT, NPARAM, NU};
use nalgebra::SMatrix;
use ordered_float::NotNan;
use rand::{Rng, SeedableRng, rngs::SmallRng};
use rerun::Vec2D;

pub type Mat<const R: usize, const C: usize> = SMatrix<f32, R, C>;

fn sub_angles(a: f32, b: f32) -> f32 {
    let mut diff_angle = a - b;
    while diff_angle < -core::f32::consts::PI {
        diff_angle += 2.0 * core::f32::consts::PI;
    } 
    while diff_angle > core::f32::consts::PI {
        diff_angle -= 2.0 * core::f32::consts::PI;
    }
    diff_angle
}

fn simulate_episode(agent: &Agent, epsilon: f32, n: usize) -> Vec<(NLPendulumModel, usize, f32)> {
    let mut trajectory = Vec::new();
    let mut rng = SmallRng::from_os_rng();
    let mut state = NLPendulumModel::bottom();
    for _ in 0..n {
        let action = if rng.random_bool(epsilon as f64) {
            rng.random_range(0..NU)
        } else {
            agent.best_action(&state.as_vec())
        };

        let u = cont_u(action)*1.0;
        let cur_state = state.clone();
        for _ in 0..10 {
            state.step(u as f32, 0.01 / 10.0);
        }

        trajectory.push((cur_state, action, reward(&state)));
    }
    trajectory
}

fn reward(state: &NLPendulumModel) -> f32 {
    assert!(state.theta.abs() < 4.0);
    if state.theta.abs() < 0.1 {
        50.0*(-state.thetadot.abs()).exp()
    } else {
        sub_angles(PI, state.theta).powi(2)
    }
}

fn main() {
    let mut agent = Agent::new();

    let learning_rate = 1e-3;
    let discount = 0.95;

    for _ in 0..1000 {
        let n = (10.0 / DT).round() as usize;
        let trajectory = simulate_episode(&agent, 0.3, n);
        let mut tot_reward = 0.0;

        let mut gt = Vec::new();
        for reward in trajectory[0..trajectory.len()].iter().map(|a|a.2).rev() {
            gt.push(reward + discount * gt.last().unwrap_or(&0.0));
        }
        gt.reverse();


        for i in 0..trajectory.len() {
            let (state, _action, _) = trajectory[i].clone();
            let g = gt[i];

            // agent.parameters -= grad * error * learning_rate / n as f32;
        }
        println!("{:.5}", tot_reward / n as f32);
    }

    let rec = rerun::RecordingStreamBuilder::new("pendulum_rl")
        .connect_grpc()
        .unwrap();

    let mut tot_reward = 0.0;

    // agent.parameters = [-0.0017227157,0.03166425,0.016089529,0.01576941,0.038933054].into();

    for (i, &(state, action, reward)) in simulate_episode(&agent, 0.0, (10.0/DT as f32).round() as usize).iter().enumerate() {
        rec.set_time_sequence("step", i as i64);

        rec.log(
            "stang",
            &rerun::LineStrips2D::new([[
                Vec2D::new(0.0, 0.0),
                Vec2D::new(-state.theta.sin() as f32, -state.theta.cos() as f32),
            ]]),
        )
        .unwrap();

        let u = cont_u(action);
        rec.log("omegaf", &rerun::Scalars::single(state.omegaf as f64)).unwrap();
        rec.log("theta", &rerun::Scalars::single(state.theta as f64)).unwrap();
        rec.log("thetadot", &rerun::Scalars::single(state.thetadot as f64)).unwrap();
        rec.log("control", &rerun::Scalars::single(u as f64)).unwrap();
        rec.log("reward", &rerun::Scalars::single(reward as f64)).unwrap();

        tot_reward += reward;
        println!(
            "{:<12.5} {:.5} {:.5} {} {:.5} {:.5} {:.5}",
            state.omegaf,
            state.theta,
            state.thetadot,
            action,
            u,
            reward,
            agent.eval(&state.as_vec())
        );
    }
    println!("{}", tot_reward / 200.0);
    
    print!("[");
    for param in &agent.parameters {
        print!("{},", param);
    }
    println!("]");

}
