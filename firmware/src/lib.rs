#![no_std]
#![no_main]
extern crate alloc;

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

use embassy_rp::peripherals::PIO0;
use embassy_rp::pio;
use embassy_rp::bind_interrupts;

pub mod assign;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

pub mod server;
pub mod entrypoints;
pub mod encoder;
pub mod motor;