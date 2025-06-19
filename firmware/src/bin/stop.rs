#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::Pwm;
use embassy_time::Timer;
use firmware::motor::NidecMotor;

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
pub async fn entrypoint(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let dir_pin = Output::new(p.PIN_8, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE3, p.PIN_7, Default::default());
    let mut motor1 = NidecMotor::new(dir_pin, motor_pwm);
    motor1.set_output(0.0);

    let dir_pin = Output::new(p.PIN_14, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, Default::default());
    let mut motor2 = NidecMotor::new(dir_pin, motor_pwm);
    motor2.set_output(0.0);

    let _enable_pin = Output::new(p.PIN_11, Level::High);

    loop {
        Timer::after_millis(1).await;
    }
}
