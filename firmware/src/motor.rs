use core::sync::atomic::{self, AtomicI32};

use alloc::rc::Rc;
use embassy_rp::{gpio::{Input, Output}, pwm::{self, Pwm, SetDutyCycle}};

pub struct NidecMotor {
    pwm: Pwm<'static>,
    dir_pin: Output<'static>,
    pub output: f32,
    counter: Rc<AtomicI32>,
}

const TOP: u16  = 200;

#[embassy_executor::task(pool_size=8)]
async fn tracker(mut encoder_pin: Input<'static>, counter: Rc<AtomicI32>) {
    loop {
        encoder_pin.wait_for_falling_edge().await;
        counter.fetch_add(1, atomic::Ordering::Relaxed);
    }
}

impl NidecMotor {
    /// `output` should be a value in the interval [-1.0, 1.0]
    pub fn set_output(&mut self, output: f32) {
        self.output = output;
        let mag = (1.0-output.abs().clamp(0.0, 1.0))*TOP as f32;

        if output < 0.0 {
            self.dir_pin.set_high();
        } else {
            self.dir_pin.set_low();
        }
        self.pwm.set_duty_cycle(mag as u16).unwrap();
    }

    pub fn ticks(&self) -> i32 {
        self.counter.load(atomic::Ordering::Relaxed)
    }

    pub fn new(
        dir_pin: Output<'static>,
        mut pwm: Pwm<'static>,
    ) -> Self {
        let counter = Rc::new(AtomicI32::new(0));
        // unwrap!(spawner.spawn(tracker(encoder_pin, counter.clone())));

        let mut pwm_config = pwm::Config::default();
        pwm_config.divider = 150.into();
        pwm_config.top = TOP;
        pwm_config.compare_b = TOP;
        pwm.set_config(&pwm_config);
        NidecMotor {
            dir_pin,
            pwm,
            output: 0.0,
            counter,
        }
    }
}
