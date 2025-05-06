#![no_std]
#![no_main]

use core::sync::atomic::{self, AtomicU32};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::pwm;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::Pwm;
use embassy_time::Timer;
use firmware::server::{start_network, transmitter};
use firmware::split_resources;
use {defmt_rtt as _, panic_probe as _};

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Pico template"),
    embassy_rp::binary_info::rp_program_description!(c"Pico 2W template"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

static COUNTER: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn count_ticks(mut pin1: Input<'static>, pin2: Input<'static>) -> ! {
    loop {
        pin1.wait_for_high().await;
        pin1.wait_for_low().await;
        COUNTER.fetch_add(1, atomic::Ordering::Relaxed);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    unwrap!(spawner.spawn(count_ticks(Input::new(p.PIN_11, Pull::Down), Input::new(p.PIN_12, Pull::Down))));
    
    use firmware::assign::*;
    let r = split_resources!(p);
    let (stack, control) = start_network(r.net, &spawner).await;

    unwrap!(spawner.spawn(transmitter(stack, control)));

    let mut pwm_config = pwm::Config::default();
    pwm_config.divider = 150.into();
    pwm_config.top = 200;
    pwm_config.compare_b = 200;
    let mut motor_pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, pwm_config.clone());
    let mut enable_pin = Output::new(p.PIN_10, Level::Low);
    Timer::after_millis(500).await;
    enable_pin.set_high();

    // let forward_pin = Output::new(pin, initial_output)

    loop {
        Timer::after_millis(1000).await;
        pwm_config.compare_b = 150;
        motor_pwm.set_config(&pwm_config);
        info!("set to {}/{}", pwm_config.compare_b, pwm_config.top);
        Timer::after_millis(1000).await;
        pwm_config.compare_b = 200;
        motor_pwm.set_config(&pwm_config);
        info!("set to {}/{}", pwm_config.compare_b, pwm_config.top);
        // info!("ticks: {}", COUNTER.load(atomic::Ordering::Relaxed));
    }
}


