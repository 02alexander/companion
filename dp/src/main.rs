use std::f64::{INFINITY, consts::PI};

use nalgebra::{SMatrix, Transform};
use rand::Rng;

pub type Mat<const R: usize, const C: usize> = SMatrix<f64, R, C>;

#[derive(Clone, Copy, Debug)]
struct Model {
    omegaf: f64,
    theta: f64,
    thetadot: f64,
}

struct Discretizer {
    max: f64,
    min: f64,
    n: usize,
}

impl Discretizer {
    pub fn new(max: f64, min: f64, n: usize) -> Discretizer {
        assert!(n > 1);
        Discretizer { max, min, n }
    }
    pub fn discretize(&self, x: f64) -> usize {
        let interval_size = (self.max - self.min) / (self.n as f64 - 1.0);
        (((x - self.min) / interval_size).round() as isize).clamp(0, self.n as isize - 1) as usize
    }
    pub fn inv(&self, n: usize) -> f64 {
        let interval_size = (self.max - self.min) / (self.n as f64 - 1.0);
        interval_size * n as f64 + self.min
    }
}

impl Model {
    pub fn step(&mut self, u: f64, dt: f64) {
        // let ir = 0.1;
        let ir = 0.03320426557380722 * 1.15;
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
    }

    fn discretized_state(
        &self,
        omegafdisc: &Discretizer,
        thetadisc: &Discretizer,
        thetadotdisc: &Discretizer,
    ) -> [usize; 3] {
        [
            omegafdisc.discretize(self.omegaf),
            thetadisc.discretize(self.theta),
            thetadotdisc.discretize(self.thetadot),
        ]
    }

    pub fn reward(&self) -> f64 {
        let f: Mat<1, 3> = [[-0.00582265], [-8.58642], [-1.02539]].into();
        let x: Mat<3, 1> = [self.omegaf, self.theta, self.thetadot].into();
        let u = (-f * x)[0];

        if self.theta.abs() < 0.3 {
            2.0 - u.clamp(-1.0, 1.0).abs()
        } else {
            0.0
        }
    }
}

struct Table {
    d1: Discretizer,
    d2: Discretizer,
    d3: Discretizer,
    du: Discretizer,

    // Q(s,a)
    rewards: Vec<Vec<Vec<f64>>>,
    transitions: Vec<Vec<Vec<Vec<[usize; 3]>>>>,
    values: Vec<Vec<Vec<f64>>>,
    new_values: Vec<Vec<Vec<f64>>>,
}

impl Table {
    pub fn new() -> Table {
        let d1 = Discretizer {
            min: -400.0,
            max: 400.0,
            n: 41,
        };

        let d2 = Discretizer {
            min: -PI,
            max: PI,
            n: 161,
        };

        let d3 = Discretizer {
            min: -60.0,
            max: 60.0,
            n: 61,
        };
        let du = Discretizer {
            min: -1.0,
            max: 1.0,
            n: 61,
        };
        let mut transitions = vec![vec![vec![Vec::new(); d3.n]; d2.n]; d1.n];
        let mut values = vec![vec![vec![0.0; d3.n]; d2.n]; d1.n];
        let mut rewards = vec![vec![vec![0.0; d3.n]; d2.n]; d1.n];

        let dt = 0.1;
        let ndt = 20;

        for s1 in 0..d1.n {
            for s2 in 0..d2.n {
                for s3 in 0..d3.n {
                    let start_state = Model {
                        omegaf: d1.inv(s1),
                        theta: d2.inv(s2),
                        thetadot: d3.inv(s3),
                    };
                    rewards[s1][s2][s3] = start_state.reward();
                    values[s1][s2][s3] = start_state.reward();
                    for ud in 0..du.n {
                        let u = du.inv(ud);

                        let mut end_state = start_state.clone();
                        for _ in 0..ndt {
                            end_state.step(u, dt / ndt as f64);
                        }

                        let dstate = end_state.discretized_state(&d1, &d2, &d3);

                        transitions[s1][s2][s3].push(dstate);
                    }
                }
            }
        }

        Table {
            d1,
            d2,
            d3,
            du,
            rewards,
            transitions,
            new_values: values.clone(),
            values,
        }
    }

    pub fn rsa(&self, s: &[usize; 3], a: usize) -> f64 {
        let sprim = self.transitions[s[0]][s[1]][s[2]][a];
        self.rewards[sprim[0]][sprim[1]][sprim[2]] - self.rewards[s[0]][s[1]][s[2]]
    }

    pub fn cvalue(&self, cs: &Model) -> f64 {
        let s = cs.discretized_state(&self.d1, &self.d2, &self.d3);
        self.values[s[0]][s[1]][s[2]]
    }

    pub fn creward(&self, cs: &Model) -> f64 {
        let s = cs.discretized_state(&self.d1, &self.d2, &self.d3);
        self.rewards[s[0]][s[1]][s[2]]
    }

    pub fn cstate(&self, s: [usize; 3]) -> Model {
        Model {
            omegaf: self.d1.inv(s[0]),
            theta: self.d2.inv(s[1]),
            thetadot: self.d3.inv(s[2]),
        }
    }

    pub fn best_action(&self, cs: &Model) -> f64 {
        let s = cs.discretized_state(&self.d1, &self.d2, &self.d3);
        let mut best_action = None;
        let mut best_er = -INFINITY;
        for action in 0..self.du.n {
            let sprim = self.transitions[s[0]][s[1]][s[2]][action];
            let er = self.values[sprim[0]][sprim[1]][sprim[2]];
            if er > best_er {
                best_action = Some(action);
                best_er = er;
            }
        }
        self.du.inv(best_action.unwrap())
    }

    pub fn update(&mut self, learning_rate: f64, discount: f64) {
        for s1 in 0..self.d1.n {
            for s2 in 0..self.d2.n {
                for s3 in 0..self.d3.n {
                    let mut best_er = -INFINITY;

                    if self.rewards[s1][s2][s3] != 0.0 {
                        continue;
                    }

                    for action in 0..self.du.n {
                        let sprim = self.transitions[s1][s2][s3][action];
                        let er = discount * self.values[sprim[0]][sprim[1]][sprim[2]];
                        best_er = best_er.max(er);
                    }
                    let target = best_er;
                    // self.values[s1][s2][s3] = target;
                    self.new_values[s1][s2][s3] =
                        (1.0 - learning_rate) * self.values[s1][s2][s3] + learning_rate *target;
                }
            }
        }
        std::mem::swap(&mut self.new_values, &mut self.values);
    }
}

fn main() {
    let mut table = Table::new();

    let bottom_right = Model {
        omegaf: 0.0,
        theta: -PI + 0.4,
        thetadot: 0.0,
    };
    let bottom_left = Model {
        omegaf: 0.0,
        theta: -PI - 0.4,
        thetadot: 0.0,
    };

    let bottom = Model {
        omegaf: 0.0,
        theta: -PI ,
        thetadot: 0.0,
    };

    let top_left = Model {
        omegaf: 0.0,
        theta: 0.33,
        thetadot: 0.0,
    };

    let top_right = Model {
        omegaf: 0.0,
        theta: -0.33,
        thetadot: 0.0,
    };

    let top = Model {
        omegaf: 0.0,
        theta: 0.0,
        thetadot: 0.0,
    };

    // println!("{}", table.creward(&top));

    let sbottom = top_left.discretized_state(&table.d1, &table.d2, &table.d3);
    println!(
        "top-interval {:?} - {:?}",
        table.d2.discretize(-0.2),
        table.d2.discretize(0.2)
    );

    println!(
        "{:?}",
        table.transitions[sbottom[0]][sbottom[1]][sbottom[2]]
    );

    for i in 0..400 {
        println!("{i}");
        println!(
            "{} {} {} {}",
            table.cvalue(&bottom_left),
            table.cvalue(&bottom_right),
            table.cvalue(&top_left),
            table.cvalue(&top_right)
        );
        println!(
            "{} {} {} {}",
            table.best_action(&bottom_left),
            table.best_action(&bottom_right),
            table.best_action(&top_left),
            table.best_action(&top_right)
        );
        println!("");
        let tot = table.d1.n * table.d2.n * table.d3.n;
        let mut cnt = 0;
        for s1 in 0..table.d1.n {
            for s2 in 0..table.d2.n {
                for s3 in 0..table.d3.n {
                    if table.values[s1][s2][s3] != 0.0 {
                        cnt += 1;
                    }
                }
            }
        }
        println!("{cnt}/{tot}");

        table.update(0.1, 0.9);
    }

    println!("============================================================");
    let mut state = bottom;
    for i in 0..500 {
        println!("{i}");
        let u = table.best_action(&state);
        for _ in 0..20 {
            state.step(u, 0.01/20.0);
            println!("{:.5} {:.5} {:.5} {:.5}", state.theta, state.thetadot, u, table.creward(&state));
        }
    }

}
