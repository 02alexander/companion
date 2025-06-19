use std::{f32::{consts::PI, INFINITY}, fs::OpenOptions, io::Write};

use common::dp::{cont, cont_u, disc_state, NU, NXS};
use nalgebra::SMatrix;

pub type Mat<const R: usize, const C: usize> = SMatrix<f32, R, C>;

#[derive(Clone, Copy, Debug)]
struct Model {
    omegaf: f32,
    theta: f32,
    thetadot: f32,
}

fn cont_state(dstate: [usize; 3]) -> Model {
    Model {
        omegaf: cont(-400.0, 400.0, NXS[0], dstate[0]),
        theta: cont(0.0, PI, NXS[1], dstate[1]),
        thetadot: cont(-20.0, 20.0, NXS[2], dstate[2])
    }
}

impl Model {
    pub fn step(&mut self, u: f32, dt: f32) {
        let ir = 0.03320426557380722 * 2.0;
        let g = 9.81;
        let r = 0.145;
        let domegaf = dt * (330.0 * u - self.omegaf) / 0.7;
        let dtheta = dt * self.thetadot;
        let dthetadot = dt
            * (-ir * (330.0 * u - self.omegaf) / 0.7 - 0.2 * self.thetadot
                + g / r * self.theta.sin());

        self.omegaf += domegaf;
        self.theta += dtheta;
        self.thetadot += dthetadot;

        if self.theta > PI {
            self.theta -= 2.0 * PI;
        } else if self.theta < -PI {
            self.theta += 2.0 * PI;
        }

        if self.theta < 0.0 {
            self.theta = -self.theta;
            self.omegaf = -self.omegaf;
            self.thetadot = -self.thetadot;
        }
    }

    pub fn as_vec(&self) -> Mat<3,1> {
        [self.omegaf, self.theta, self.thetadot].into()
    }

    pub fn as_disc(&self) -> [usize; 3] {
        disc_state(self.as_vec().into())
    }
    
    pub fn reward(&self) -> f32 {
        let f: Mat<1, 3> = [[-0.00582265], [-8.58642], [-1.02539]].into();
        let x: Mat<3, 1> = [self.omegaf, self.theta, self.thetadot].into();
        let u = (-f * x)[0];

        if self.theta.abs() < 0.2 {
            2.0 - u.clamp(-1.0, 1.0).abs()
        } else {
            0.0
        }
    }
}

struct Table {
    // Q(s,a)
    rewards: Vec<Vec<Vec<f32>>>,
    transitions: Vec<Vec<Vec<Vec<[usize; 3]>>>>,
    values: Vec<Vec<Vec<f32>>>,
    new_values: Vec<Vec<Vec<f32>>>,
}

impl Table {
    pub fn new() -> Table {
        let mut transitions = vec![vec![vec![Vec::new(); NXS[2]]; NXS[1]]; NXS[0]];
        let mut values = vec![vec![vec![0.0; NXS[2]]; NXS[1]]; NXS[0]];
        let mut rewards = vec![vec![vec![0.0; NXS[2]]; NXS[1]]; NXS[0]];

        let dt = 0.1;
        let ndt = 20;

        for s1 in 0..NXS[0] {
            for s2 in 0..NXS[1] {
                for s3 in 0..NXS[2] {
                    let ds = [s1, s2, s3];
                    let start_state = cont_state(ds);
                    rewards[s1][s2][s3] = start_state.reward();
                    values[s1][s2][s3] = start_state.reward();
                    for ud in 0..NU {
                        let u = cont_u(ud);

                        let mut end_state = start_state.clone();
                        for _ in 0..ndt {
                            end_state.step(u, dt / ndt as f32);
                        }

                        let dstate = end_state.as_disc();

                        transitions[s1][s2][s3].push(dstate);
                    }
                }
            }
        }

        Table {
            rewards,
            transitions,
            new_values: values.clone(),
            values,
        }
    }

    pub fn rsa(&self, s: &[usize; 3], a: usize) -> f32 {
        let sprim = self.transitions[s[0]][s[1]][s[2]][a];
        self.rewards[sprim[0]][sprim[1]][sprim[2]] - self.rewards[s[0]][s[1]][s[2]]
    }

    pub fn cvalue(&self, cs: &Model) -> f32 {
        let s = cs.as_disc();
        self.values[s[0]][s[1]][s[2]]
    }

    pub fn creward(&self, cs: &Model) -> f32 {
        let s = cs.as_disc();
        self.rewards[s[0]][s[1]][s[2]]
    }

    pub fn dbest_action(&self, s: [usize; 3]) -> usize {
        let mut best_action = None;
        let mut best_er = -INFINITY;
        for action in 0..NU {
            let sprim = self.transitions[s[0]][s[1]][s[2]][action];
            let er = self.values[sprim[0]][sprim[1]][sprim[2]];
            if er > best_er {
                best_action = Some(action);
                best_er = er;
            }
        }
        best_action.unwrap()
    }


    pub fn best_action(&self, cs: &Model) -> f32 {
        let s = cs.as_disc();
        let mut best_action = None;
        let mut best_er = -INFINITY;
        for action in 0..NU {
            let sprim = self.transitions[s[0]][s[1]][s[2]][action];
            let er = self.values[sprim[0]][sprim[1]][sprim[2]];
            if er > best_er {
                best_action = Some(action);
                best_er = er;
            }
        }
        cont_u(best_action.unwrap())
    }

    pub fn update(&mut self, learning_rate: f32, discount: f32) {
        for s1 in 0..NXS[0] {
            for s2 in 0..NXS[1] {
                for s3 in 0..NXS[2] {
                    let mut best_er = -INFINITY;


                    let r = self.rewards[s1][s2][s3];
                    // if self.rewards[s1][s2][s3] != 0.0 {
                    //     continue;
                    // }

                    for action in 0..NU {
                        let sprim = self.transitions[s1][s2][s3][action];
                        let er = r + discount * self.values[sprim[0]][sprim[1]][sprim[2]];
                        best_er = best_er.max(er);
                    }
                    self.new_values[s1][s2][s3] =
                        (1.0 - learning_rate) * self.values[s1][s2][s3] + learning_rate *best_er;
                }
            }
        }
        std::mem::swap(&mut self.new_values, &mut self.values);
    }
}

fn main() {
    let mut table = Table::new();

    let bottom = Model {
        omegaf: 0.0,
        theta: -PI ,
        thetadot: 0.0,
    };
    let bottom_left = Model {
        omegaf: 0.0,
        theta: -PI - 0.4 ,
        thetadot: 0.0,
    };
    let bottom_right = Model {
        omegaf: 0.0,
        theta: -PI + 0.4 ,
        thetadot: 0.0,
    };

    for i in 0..1000 {
        println!("{i}");
        let tot = NXS[0] * NXS[1] * NXS[2];
        let mut cnt = 0;
        println!("{}", cont_u(table.best_action(&bottom_left) as usize));
        println!("{}", cont_u(table.best_action(&bottom_right) as usize));
        println!("{}", table.cvalue(&bottom_left));

        for s1 in 0..NXS[0] {
            for s2 in 0..NXS[1] {
                for s3 in 0..NXS[2] {
                    if table.values[s1][s2][s3] != 0.0 {
                        cnt += 1;
                    }
                }
            }
        }
        println!("{cnt}/{tot}");
        println!("");

        table.update(0.2, 0.9);
    }

    println!("============================================================");
    let mut state = bottom;
    for i in 0..500 {
        println!("{i}");
        let u = table.best_action(&state);
        println!("{:.5} {:.5} {:.5} {:.5}", state.theta, state.thetadot, u, table.creward(&state));
        for _ in 0..20 {
            state.step(u, 0.01/20.0);
        }
    }

    let mut code_array = String::new();
    use std::fmt::Write;
    write!(code_array, "[\n").unwrap();
    for s1 in 0..NXS[0] {
        write!(code_array, "[").unwrap();
        for s2 in 0..NXS[1] {
            write!(code_array, "[").unwrap();
            for s3 in 0..NXS[2] {
                let action = table.dbest_action([s1, s2, s3]);
                write!(code_array, "{},", action).unwrap();
            }
            write!(code_array, "],").unwrap();
        }
        write!(code_array, "],\n").unwrap();
    }
    write!(code_array, "]").unwrap();

    let rust_code = format!("
pub const DP_TABLE: [[[u8; {}]; {}]; {}] = {};\n
", NXS[2], NXS[1], NXS[0], code_array);
    
    let mut file = OpenOptions::new().write(true).truncate(true).create(true).open("/home/alexander/prog/companion/firmware/src/dptable.rs").unwrap();
    file.write_all(rust_code.as_bytes()).unwrap();
}
