use embassy_rp::{bind_interrupts, i2c, peripherals::I2C0};

pub mod benchtest;
pub mod test_encoder;
pub mod main;


bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});
