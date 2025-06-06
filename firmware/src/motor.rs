use core::sync::atomic::{self, AtomicI32};

use alloc::rc::Rc;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_rp::{gpio::{Input, Output}, pwm::{Pwm, SetDutyCycle}};

pub struct NidecMotor {
    pwm: Pwm<'static>,
    dir_pin: Output<'static>,
    pub output: f32,
    counter: Rc<AtomicI32>,
}

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
        let min_mag = 5.0 / 200.0;
        let mag = output.abs() * ((1.0 - min_mag)*200.0);

        if output < 0.0 {
            self.dir_pin.set_high();
        } else {
            self.dir_pin.set_low();
        }
        self.pwm.set_duty_cycle((mag as u16).clamp(5, 200)).unwrap();
    }

    pub fn ticks(&self) -> i32 {
        self.counter.load(atomic::Ordering::Relaxed)
    }

    pub fn new(
        spawner: &Spawner,
        dir_pin: Output<'static>,
        encoder_pin: Input<'static>,
        pwm: Pwm<'static>,
    ) -> Self {
        let counter = Rc::new(AtomicI32::new(0));
        unwrap!(spawner.spawn(tracker(encoder_pin, counter.clone())));
        assert_eq!(pwm.max_duty_cycle(), 200);
        NidecMotor {
            dir_pin,
            pwm,
            output: 0.0,
            counter,
        }
    }
}
