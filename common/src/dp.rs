use core::f32::consts::PI;

use nalgebra::SMatrix;

type Mat<const R: usize, const C: usize> = SMatrix<f32, R, C>;

pub const NXS: [usize; 3] = [41, 21, 31];
pub const NU: usize = 41;

pub fn disc(min: f32, max: f32, n: usize, x: f32) -> usize {
    let interval_size = (max - min) / (n as f32 - 1.0);
    (((x - min) / interval_size) as isize).clamp(0, n as isize - 1) as usize
}

pub fn cont(min: f32, max: f32, n: usize, k: usize) -> f32 {
    let interval_size = (max - min) / (n as f32 - 1.0);
    interval_size * k as f32 + min    
}

pub fn disc_state(state: Mat<3,1>) -> [usize; 3] {
    [
        disc(-400.0, 400.0, NXS[0], state[0]),
        disc(0.0, PI, NXS[1], state[1]),
        disc(-20.0, 20.0, NXS[2], state[2])
    ]
}

pub fn cont_u(du: usize) -> f32 {
    cont(-1.0, 1.0, NU, du)
}

pub fn disc_u(u: f32) -> usize {
    disc(-1.0, 1.0, NU, u)
}
