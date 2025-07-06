#![no_std]
#![no_main]

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

use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c;
use embassy_rp::peripherals;
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio;

pub struct Netresources {
    pub pwr: Peri<'static, peripherals::PIN_23>,
    pub cs: Peri<'static, peripherals::PIN_25>,
    pub pio: Peri<'static, peripherals::PIO0>,
    pub dio: Peri<'static, peripherals::PIN_24>,
    pub spi_clk: Peri<'static, peripherals::PIN_29>,
    pub dma: Peri<'static, peripherals::DMA_CH0>,
}

bind_interrupts!(pub struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

pub mod encoder;
pub mod motor;
pub mod server;
