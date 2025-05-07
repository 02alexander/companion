#![no_std]
#![no_main]

use embassy_rp::peripherals::PIO0;
use embassy_rp::pio;
use embassy_rp::bind_interrupts;

pub mod assign;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

pub mod server;
pub mod entrypoints;