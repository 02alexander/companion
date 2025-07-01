#![no_std]
#![no_main]

use core::f32::consts::PI;

use common::dp::{cont_u, Agent};
use common::filter::{pendulum_model, Mat, Model, NLModel, EKF, RADIUS};
use embassy_rp::i2c::I2c;
use embassy_rp::pac::uart::regs::Uartcr;
use firmware::dptable::DP_TABLE;
use firmware::encoder::{MagneticEncoder, RotaryEncoder};
use firmware::motor::NidecMotor;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Instant, Ticker};

use {defmt_rtt as _, panic_probe as _};


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

#[derive(PartialEq, Clone, Copy)]
enum BalancingState {
    Swinging,
    Chilling,
    Balancing,
}

#[embassy_executor::main]
pub async fn entrypoint(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let dir_pin = Output::new(p.PIN_8, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE3, p.PIN_7, Default::default());
    let mut motor = NidecMotor::new(dir_pin, motor_pwm);
    let mut wheel_encoder_pin = Input::new(p.PIN_6, Pull::Up);
    motor.set_output(0.0);

    let _enable_pin = Output::new(p.PIN_11, Level::High);

    info!("Entering loop...");


    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let i2c = I2c::new_async(
        p.I2C0,
        scl,
        sda,
        firmware::Irqs,
        Default::default(),
    );
    let mut encoder = MagneticEncoder { channel: i2c };
    let ref_angle = -1.8149946;
    let ref_angle = sub_angles(-1.8149946, PI);
    // let ref_angle = encoder.rotation().await.unwrap();
    info!("ref_angle = {}", ref_angle);

    let mut prev_angles = [0.0; 2000];

    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut ekf = EKF::from_model(NLModel {dt: 0.01});
    ekf.x[1] = PI;
    
    let mut agent = Agent::new();
    // agent.parameters = [-0.040831022,0.0051453677,0.23291656,-1.8110611,0.58743167,].into();

    let mut prev_state = BalancingState::Swinging;
    let mut state = BalancingState::Swinging;

    let mut last_meas = None;

    loop {
        ticker.next().await;

        ekf.time_update(motor.output);

        if let Ok(raw_angle) = encoder.rotation().await {
            let angle = sub_angles(raw_angle, ref_angle);
            last_meas = Some(angle);
            let pred = ekf.x[1];            
            ekf.measurment_update_from_error([sub_angles(angle, pred)].into());
        }

        while ekf.x[1] < -PI {
            ekf.x[1] += 2.0*PI
        }
        while ekf.x[1] > PI {
            ekf.x[1] -= 2.0*PI
        }
        
        let f: Mat<1,3> = [[-0.00582551],[-8.00347],[-0.967164]].into();
        // let top_energy = RADIUS*9.81;
        // let cur_energy = RADIUS*9.81*libm::cosf(ekf.x[1]) + RADIUS*ekf.x[2]*RADIUS*ekf.x[2]/2.0;
        // info!("{} {}", cur_energy, ekf.x[2]);
        // continue;

        state = match state {
            BalancingState::Swinging => {
            

                let top_energy = RADIUS*9.81;
                let cur_energy = RADIUS*9.81*libm::cosf(ekf.x[1]) + RADIUS*ekf.x[2]*RADIUS*ekf.x[2]/2.0;

                // info!("{} {}", cur_energy, top_energy);
                if ekf.x[0].abs() > 330.0*0.2 {
                    motor.set_output(ekf.x[2].signum()*0.15);
                } else if cur_energy < top_energy {
                    // info!("Undershooting");
                    motor.set_output(-ekf.x[2].signum()*0.2);
                    // info!("Enough to reacht top! {} {} {}", ekf.x[0], ekf.x[1], ekf.x[2]);
                } else if cur_energy > top_energy {
                    // info!("Overshooting");
                    motor.set_output(ekf.x[2].signum()*0.2);
                } else {
                    motor.set_output(ekf.x[0] / 330.0);
                }
                
                let u = (-f * ekf.x)[0];
                if sub_angles(ekf.x[1],0.0).abs() < 0.15 {
                    // info!("u = {}", u);
                }
                if sub_angles(ekf.x[1],0.0).abs() < 0.2 && u.abs() < 3.0 {
                    BalancingState::Balancing
                } else {
                    BalancingState::Swinging
                }
            },
            BalancingState::Chilling => {
                let top_energy = RADIUS*9.81;
                let bot_energy = -RADIUS*9.81;
                let cur_energy = RADIUS*9.81*libm::cosf(ekf.x[1]) + RADIUS*ekf.x[2]*RADIUS*ekf.x[2]/2.0;
                
                let a = 0.7;
                if cur_energy > a*top_energy + (1.0-a)*bot_energy {
                    motor.set_output(ekf.x[2].signum()*0.3);
                    BalancingState::Chilling
                } else {
                    BalancingState::Swinging
                }
            },
            BalancingState::Balancing => {
                let f: Mat<1,3> = [[-0.00582551],[-8.00347],[-0.967164]].into();
                let u = (-f * ekf.x)[0];
                info!("{} {} {}", f[0]*ekf.x[0], f[1]*ekf.x[1], f[2]*ekf.x[2]);
                motor.set_output(u.clamp(-1.0, 1.0));
                if sub_angles(ekf.x[1],0.0).abs() > 0.25 {
                    BalancingState::Chilling
                } else {
                    BalancingState::Balancing
                }
            }
        };
        if prev_state != state {
            match state {
                BalancingState::Swinging => info!("Swinging"),
                BalancingState::Chilling => info!("Chilling"),
                BalancingState::Balancing => info!("Balancing"),
            }
        }
        prev_state = state;

        
    }
}
