
use common::LogMessage;
use core::sync::atomic::{self, AtomicI32, AtomicI16};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm;
use embassy_rp::pwm::Pwm;
use embassy_time::Timer;
use crate::server::{start_network, transmitter};
use crate::*;
use heapless::mpmc::Q4;

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

static COUNTER: AtomicI32 = AtomicI32::new(0);
static ABS_TICKS: AtomicI32 = AtomicI32::new(0);
static PWM_REF: AtomicI16 = AtomicI16::new(0);


#[embassy_executor::task]
async fn track_encoder(mut pin1: Input<'static>, mut pin2: Input<'static>) -> ! {
    let mut prev_state = (pin1.is_high(), pin2.is_high());
    loop {
        embassy_futures::select::select(pin1.wait_for_any_edge(), pin2.wait_for_any_edge()).await;


        // Keeps track of the relative position of the motor.
        let cur_state = (pin1.is_high(), pin2.is_high());
        if prev_state == (true, true) {
            if cur_state == (true, false) {
                COUNTER.fetch_sub(1, atomic::Ordering::Relaxed);
            }
        }
        if prev_state == (true, false) {
            if cur_state == (true, true) {
                COUNTER.fetch_add(1, atomic::Ordering::Relaxed);
            }
        }

        // Tracks the absolute value of its change, using only one pin.
        if prev_state.0 != cur_state.0 {
            ABS_TICKS.fetch_add(1, atomic::Ordering::Relaxed);
        }

        prev_state = cur_state;
    }
}

#[embassy_executor::task]
async fn logger() {
    loop {
        let msg = LogMessage {
            ticks: COUNTER.load(atomic::Ordering::Relaxed),
            pwm: PWM_REF.load(atomic::Ordering::Relaxed),
            abs_ticks: ABS_TICKS.load(atomic::Ordering::Relaxed),
        };
        // info!("{}", msg.ticks);
        let _ = MSG_QUEUE.enqueue(msg);

        Timer::after_millis(10).await;
    }
}


struct RNG {
    state: u64,
}

impl RNG {
    pub fn new(seed: u64) -> Self {
        RNG { state: seed }
    }

    pub fn randint(&mut self) -> u64 {
        self.state ^= self.state >> 12;
        self.state ^= self.state << 25;
        self.state ^= self.state >> 27;
        self.state.wrapping_mul(0x2545F4914F6CDD1D)
    }
}

static MSG_QUEUE: Q4<LogMessage> = Q4::new();

pub async fn entrypoint(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize position tracker and logger.
    let encoder_pin = Input::new(p.PIN_12, Pull::Down);
    unwrap!(spawner.spawn(track_encoder(
        encoder_pin,
        Input::new(p.PIN_13, Pull::Down)
    )));
    unwrap!(spawner.spawn(logger()));

    // Initialize network server.
    use crate::assign::*;
    let r = split_resources!(p);
    let (stack, control) = start_network(r.net, &spawner).await;
    info!("network started!");
    unwrap!(spawner.spawn(transmitter(stack, control, &MSG_QUEUE)));
    
    
    let mut pwm_config = pwm::Config::default();
    let top = 200;
    pwm_config.divider = 150.into();
    pwm_config.top = top;
    pwm_config.compare_b = 5;
    let mut motor_pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, pwm_config.clone());
    motor_pwm.set_config(&pwm_config);
    PWM_REF.store(pwm_config.compare_b as i16, atomic::Ordering::Relaxed);
    let mut direction_pin = Output::new(p.PIN_14, Level::Low);
    let enable_pin = Output::new(p.PIN_11, Level::High);

    let mut rng = RNG::new(12345789012345678901);
    
    info!("Entering loop...");
    Timer::after_millis(1000).await;
    loop {

        direction_pin.set_high();
        pwm_config.compare_b = 50;

        PWM_REF.store(-(pwm_config.compare_b as i16), atomic::Ordering::Relaxed);
        motor_pwm.set_config(&pwm_config);
        info!("set to {}/{}", -(pwm_config.compare_b as i32), pwm_config.top);
        Timer::after_millis(6000).await;

        direction_pin.set_low();
        pwm_config.compare_b = 50;
        PWM_REF.store(pwm_config.compare_b as i16, atomic::Ordering::Relaxed);
        motor_pwm.set_config(&pwm_config);
        info!("set to {}/{}", pwm_config.compare_b as i32, pwm_config.top);
        Timer::after_millis(6000).await;


        // direction_pin.set_high();
        // pwm_config.compare_b = 2;
        // PWM_REF.store(pwm_config.compare_b as i16, atomic::Ordering::Relaxed);
        // motor_pwm.set_config(&pwm_config);
        // info!("set to {}/{}", pwm_config.compare_b as i32, pwm_config.top);
        // Timer::after_millis(3000).await;


        // pwm_config.compare_b = 50;
        // PWM_REF.store(pwm_config.compare_b as i16, atomic::Ordering::Relaxed);
        // motor_pwm.set_config(&pwm_config);
        // info!("set to {}/{}", pwm_config.compare_b as i32, pwm_config.top);
        // Timer::after_millis(3000).await;



        // let max = top / 4;
        // let val = (rng.randint() % (2*max as u64 + 1)) as i32  - max as i32;
        // if val < 0 {
        //     direction_pin.set_high();
        // } else {
        //     direction_pin.set_low();
        // }

        // pwm_config.compare_b = val.abs() as u16;
        // PWM_REF.store(val.signum() as i16*pwm_config.compare_b as i16, atomic::Ordering::Relaxed);
        // motor_pwm.set_config(&pwm_config);
        // info!("set to {}/{}", val.signum()*pwm_config.compare_b as i32, pwm_config.top);
        // Timer::after_millis(3000).await;
    }
}


