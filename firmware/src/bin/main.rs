#![no_std]
#![no_main]

use core::f32::consts::PI;

use common::filter::{EKF, Mat, NLModel, RADIUS};
use cyw43::Control;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::I2c;
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Ticker, Timer};
use firmware::Netresources;
use firmware::encoder::{MagneticEncoder, RotaryEncoder};
use firmware::motor::NidecMotor;
use firmware::server::start_network;

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

#[embassy_executor::task]
async fn blinker(mut led: Control<'static>) {
    loop {
        led.gpio_set(0, true).await;
        Timer::after_millis(500).await;
        led.gpio_set(0, false).await;
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main]
pub async fn entrypoint(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let dir_pin = Output::new(p.PIN_12, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE5, p.PIN_11, Default::default());
    let mut motor = NidecMotor::new(dir_pin, motor_pwm);
    motor.set_output(0.0);

    // Initialize network server.
    let r = Netresources {
        pwr: p.PIN_23,
        cs: p.PIN_25,
        pio: p.PIO0,
        dio: p.PIN_24,
        spi_clk: p.PIN_29,
        dma: p.DMA_CH0,
    };
    let (_stack, control) = start_network(r, &spawner).await;
    unwrap!(spawner.spawn(blinker(control)));

    let sda = p.PIN_0;
    let scl = p.PIN_1;
    let i2c = I2c::new_async(p.I2C0, scl, sda, firmware::Irqs, Default::default());
    let mut encoder = MagneticEncoder { channel: i2c };
    let bottom_angle = 0.7473148 + 0.035;
    info!("bottom_angle = {}", bottom_angle);
    let ref_angle = sub_angles(bottom_angle, PI);

    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut ekf = EKF::from_model(NLModel { dt: 0.01 });
    ekf.x[1] = PI;

    let mut prev_state = BalancingState::Swinging;
    let mut state = BalancingState::Swinging;

    info!("Entering loop...");

    loop {
        ticker.next().await;

        ekf.time_update(motor.output);

        if let Ok(raw_angle) = encoder.rotation().await {
            let angle = sub_angles(raw_angle, ref_angle);
            let pred = ekf.x[1];
            ekf.measurment_update_from_error([sub_angles(angle, pred)].into());
        }

        while ekf.x[1] < -PI {
            ekf.x[1] += 2.0 * PI
        }
        while ekf.x[1] > PI {
            ekf.x[1] -= 2.0 * PI
        }

        let f: Mat<1, 3> = [[-0.00582551], [-8.00347], [-0.967164]].into();

        state = match state {
            BalancingState::Swinging => {
                let top_energy = RADIUS * 9.81;
                let cur_energy = RADIUS * 9.81 * libm::cosf(ekf.x[1])
                    + RADIUS * ekf.x[2] * RADIUS * ekf.x[2] / 2.0;

                if ekf.x[0].abs() > 330.0 * 0.2 {
                    motor.set_output(ekf.x[2].signum() * 0.15);
                } else if cur_energy < top_energy {
                    motor.set_output(-ekf.x[2].signum() * 0.2);
                } else if cur_energy > top_energy {
                    motor.set_output(ekf.x[2].signum() * 0.2);
                } else {
                    motor.set_output(ekf.x[0] / 330.0);
                }

                let u = (-f * ekf.x)[0];
                if sub_angles(ekf.x[1], 0.0).abs() < 0.2 && u.abs() < 3.0 {
                    BalancingState::Balancing
                } else {
                    BalancingState::Swinging
                }
            }
            BalancingState::Chilling => {
                let top_energy = RADIUS * 9.81;
                let bot_energy = -RADIUS * 9.81;
                let cur_energy = RADIUS * 9.81 * libm::cosf(ekf.x[1])
                    + RADIUS * ekf.x[2] * RADIUS * ekf.x[2] / 2.0;

                let boundary = 0.7;
                if cur_energy > boundary * top_energy + (1.0 - boundary) * bot_energy {
                    motor.set_output(ekf.x[2].signum() * 0.3);
                    BalancingState::Chilling
                } else {
                    BalancingState::Swinging
                }
            }
            BalancingState::Balancing => {
                let f: Mat<1, 3> = [[-0.00582551], [-8.00347], [-0.967164]].into();
                let u = (-f * ekf.x)[0];
                info!(
                    "{} {} {}",
                    f[0] * ekf.x[0],
                    f[1] * ekf.x[1],
                    f[2] * ekf.x[2]
                );
                motor.set_output(u.clamp(-1.0, 1.0));
                if sub_angles(ekf.x[1], 0.0).abs() > 0.25 {
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
