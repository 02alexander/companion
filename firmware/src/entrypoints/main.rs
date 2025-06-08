use crate::encoder::{MagneticEncoder, RotaryEncoder};
use crate::motor::NidecMotor;
use crate::server::{GOT_CONNECTION, start_network, transmitter};
use crate::*;
use common::{EKF, LogMessage, SAMPLE_TIME_MS};
use core::sync::atomic::{self, AtomicI16};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::pwm;
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::mpmc::Q4;

use {defmt_rtt as _, panic_probe as _};

static LOG_MSG_QUEUE: Q4<LogMessage> = Q4::new();

fn sub_angles(a: f32, b: f32) -> f32 {
    let mut diff_angle = a - b;
    if diff_angle < -core::f32::consts::PI {
        diff_angle += 2.0 * core::f32::consts::PI;
    } else if diff_angle > core::f32::consts::PI {
        diff_angle -= 2.0 * core::f32::consts::PI;
    }
    diff_angle
}

/// Waits for the wheel to settle down and then reads the encoder.
pub async fn get_reference<T: RotaryEncoder>(encoder: &mut T) -> Result<f32, T::Error> {
    let mut last = encoder.rotation().await?;
    'outer: loop {
        for _ in 0..8 {
            Timer::after_millis(40).await;
            let cur = encoder.rotation().await?;
            if sub_angles(cur, last).abs() > 0.01 {
                last = cur;
                continue 'outer;
            }
        }
        break;
    }
    Ok(last)
}

pub struct Controller {
    wheel_ticks: i32,
    wheel_vel: f32,
    pend_angle: f32,
    pend_vel: f32,
    ref_angle: f32,
}

impl Controller {
    pub fn new(start_ticks: i32, ref_angle: f32) -> Controller {
        Controller {
            wheel_ticks: start_ticks,
            ref_angle,
            wheel_vel: 0.0,
            pend_angle: 0.0,
            pend_vel: 0.0,
        }
    }

    pub fn step(&mut self, ticks: i32, angle: f32) -> Option<f32> {
        Some(0.0)
    }
}

pub async fn entrypoint(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // let encoder_pin = Input::new(p.PIN_12, Pull::Down);
    // let pwm_pin = p.PIN_15;
    // let pwm_slice = p.PWM_SLICE7;
    // let direction_pin = Output::new(p.PIN_14, Level::Low);

    let encoder_pin = Input::new(p.PIN_8, Pull::Down);
    let pwm_pin = p.PIN_7;
    let pwm_slice = p.PWM_SLICE3;
    let direction_pin = Output::new(p.PIN_9, Level::Low);

    let mut pwm_config = pwm::Config::default();
    pwm_config.divider = 150.into();
    pwm_config.top = 200;
    pwm_config.compare_b = 0;
    let motor_pwm = Pwm::new_output_b(pwm_slice, pwm_pin, pwm_config.clone());

    let mut motor = NidecMotor::new(&spawner, direction_pin, encoder_pin, motor_pwm);

    let _enable_pin = Output::new(p.PIN_11, Level::High);

    // Initialize network server.
    use crate::assign::*;
    let r = split_resources!(p);
    let (stack, control) = start_network(r.net, &spawner).await;
    info!("network started!");
    unwrap!(spawner.spawn(transmitter(stack, control, &LOG_MSG_QUEUE)));

    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let i2c = I2c::new_async(
        p.I2C0,
        scl,
        sda,
        crate::entrypoints::Irqs,
        Default::default(),
    );

    let mut encoder = MagneticEncoder { channel: i2c };

    let ts = Duration::from_millis(SAMPLE_TIME_MS as u64);
    let mut ticker = Ticker::every(ts);

    ticker.next().await;

    let mut last_step = Instant::now();
    let mut prev_ticks = None;

    let mut ekf = EKF::default();

    info!("Fetching rotation...");
    let ref_angle = get_reference(&mut encoder).await.unwrap();
    info!("ref_angle = {}", ref_angle);
    info!("Fetched rotation!");
    let ref_angle = -2.147573 - 0.0055;
    let ref_angle = sub_angles(ref_angle, core::f32::consts::PI);
    let mut pos_step = false;
    motor.set_output(0.0);

    info!("Waiting for connection...");
    while let None = GOT_CONNECTION.dequeue() {
        Timer::after_millis(100).await;
    }
    Timer::after_millis(2000).await;

    let start_time = Instant::now();
    info!("Entering control loop!");
    loop {
        let Ok(cur_angle) = encoder.rotation().await else {
            info!("Failed to read from encoder");
            continue;
        };
        let diff_angle = sub_angles(cur_angle, ref_angle);
        info!("angles {} - {} = {}", cur_angle, ref_angle, diff_angle);
        let cur_time = Instant::now();

        // if cur_time - start_time > Duration::from_millis(2000) {
        //     motor.set_output(0.25);
        // }

        let F: common::external::nalgebra::Matrix1x3<f32> = [
            [-16.817253264790242],
            [15.38442249442338],
            [0.0007010951026155611],
        ]
        .into();
        let u = (-F * ekf.state)[0];
        // motor.set_output(u.clamp(-0.9, 0.9));
        motor.set_output((u * 2.0).clamp(-0.95, 0.95) / 2.0 + 0.5);
        info!("{} {}", u, motor.output);

        ekf.time_update(motor.output);

        let cur_ticks = motor.ticks();

        let Some((pt, t_pt)) = prev_ticks else {
            prev_ticks = Some((cur_ticks, Instant::now()));
            continue;
        };
        let cur_t = Instant::now();
        let ticks_per_rev = 100.0;
        let rev_per_sec =
            (cur_ticks - pt) as f32 * 1e6 / (ticks_per_rev * ((cur_t - t_pt).as_micros() as f32));
        let rad_per_sec = rev_per_sec * 2.0 * core::f32::consts::PI;

        if rad_per_sec <= 10000.0 {
            ekf.measurment_update([rad_per_sec, diff_angle, 0.0].into())
                .unwrap();
        }
        prev_ticks = Some((cur_ticks, cur_t));

        let y = ekf.C * ekf.state;

        let msg = LogMessage {
            time_ms: Instant::now().as_millis(),
            wheel_velocity: y[0],
            control: motor.output,
            pend_angle: y[1],
            pend_velocity: y[2],
            sensor_pend_angle: diff_angle,
            sensor_wheel_velocity: motor.ticks() as f32,
        };

        let _ = LOG_MSG_QUEUE.enqueue(msg);

        ticker.next().await;
    }
}
