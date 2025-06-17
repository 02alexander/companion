#![no_std]
#![no_main]

use firmware::motor::NidecMotor;
use firmware::server::{start_network, transmitter, GOT_CONNECTION};
use common::{BenchMessage, LogMessage};
use firmware::split_resources;
use core::f32::consts::PI;
use core::sync::atomic::{self, AtomicI32, Ordering};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::mpmc::Q4;

use {defmt_rtt as _, panic_probe as _};

static TICKS: AtomicI32 = AtomicI32::new(0);
static ABS_TICKS: AtomicI32 = AtomicI32::new(0);

#[embassy_executor::task]
async fn track_encoder(mut pin1: Input<'static>, mut pin2: Input<'static>) -> ! {
    let mut prev_state = (pin1.is_high(), pin2.is_high());
    loop {
        embassy_futures::select::select(pin1.wait_for_any_edge(), pin2.wait_for_any_edge()).await;

        // Keeps track of the relative position of the motor.
        let cur_state = (pin1.is_high(), pin2.is_high());
        if prev_state == (true, true) {
            if cur_state == (true, false) {
                TICKS.fetch_sub(1, atomic::Ordering::Relaxed);
            }
        }
        if prev_state == (true, false) {
            if cur_state == (true, true) {
                TICKS.fetch_add(1, atomic::Ordering::Relaxed);
            }
        }

        // Tracks the absolute value of its change, using only one pin.
        if prev_state.0 && cur_state.0 {
            ABS_TICKS.fetch_add(1, atomic::Ordering::Relaxed);
        }

        prev_state = cur_state;
    }
}

#[embassy_executor::task]
async fn heartbeat() {
    loop {
        let _ = MSG_QUEUE.enqueue(LogMessage::Alive);
        Timer::after_millis(500).await;
    }
}

struct StateLogger {
    last_ticks: i32,
    last_abs_ticks: i32,
    last_instant: Instant,
}

impl StateLogger {
    fn new() -> Self {
        StateLogger {
            last_ticks: TICKS.load(atomic::Ordering::Relaxed),
            last_abs_ticks: ABS_TICKS.load(atomic::Ordering::Relaxed),
            last_instant: Instant::now(),
        }
    }
    fn log(&mut self, ticks: i32, abs_ticks: i32, control: f32) {
        let cur_instant = Instant::now();
        let dt = (cur_instant - self.last_instant).as_micros() as f32 / 1e6;

        let ticks_per_rev = 100.0;
        let rad_per_sec = 2.0 * PI * (ticks - self.last_ticks) as f32 / (ticks_per_rev * dt);

        let abs_rad_per_sec =
            2.0 * PI * (abs_ticks - self.last_abs_ticks) as f32 / (ticks_per_rev * dt);

        self.last_ticks = ticks;
        self.last_abs_ticks = abs_ticks;
        self.last_instant = cur_instant;

        let _ = MSG_QUEUE.enqueue(LogMessage::Bench(BenchMessage {
            time_ms: cur_instant.as_millis(),
            control,
            signed_rot_speed: rad_per_sec,
            abs_rot_speed: abs_rad_per_sec,
        }));
    }
}

static MSG_QUEUE: Q4<LogMessage> = Q4::new();

#[embassy_executor::main]
pub async fn entrypoint(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let encoder_a = Input::new(p.PIN_12, Pull::Down);
    let encoder_b = Input::new(p.PIN_13, Pull::Down);
    let dir_pin = Output::new(p.PIN_14, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, Default::default());
    let mut motor = NidecMotor::new(dir_pin, motor_pwm);
    motor.set_output(0.0);

    // Initialize position tracker and logger.
    unwrap!(spawner.spawn(track_encoder(encoder_a, encoder_b,)));

    //Initialize network server.
    use firmware::assign::*;
    let r = split_resources!(p);
    let (stack, control) = start_network(r.net, &spawner).await;
    info!("network started!");
    unwrap!(spawner.spawn(transmitter(stack, control, &MSG_QUEUE)));
    unwrap!(spawner.spawn(heartbeat()));

    let _enable_pin = Output::new(p.PIN_11, Level::High);

    info!("Entering loop...");
    let mut ticker = Ticker::every(Duration::from_millis(50));

    let mut state_logger = StateLogger::new();

    info!("Waiting for connection...");
    while GOT_CONNECTION.dequeue().is_none() {
        Timer::after_millis(1).await;
    }
    info!("Got connection!");

    let start_time = Instant::now();
    loop {
        info!("logging...");

        if Instant::now() - start_time > Duration::from_millis(2000) {
            motor.set_output(0.1);
        }

        state_logger.log(
            TICKS.load(Ordering::Relaxed),
            ABS_TICKS.load(Ordering::Relaxed),
            motor.output,
        );
        ticker.next().await;
    }
}
