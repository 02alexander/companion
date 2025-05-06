#![no_std]
#![no_main]

use embassy_rp::peripherals::PIO0;
use embassy_rp::pio;
use embassy_rp::bind_interrupts;

pub mod assign {
    use assign_resources::assign_resources;
    use embassy_rp::peripherals;
    assign_resources! {
        net: Netresources {
            pwr: PIN_23,
            cs: PIN_25,
            pio: PIO0,
            dio: PIN_24,
            spi_clk: PIN_29,
            dma: DMA_CH0,
        }
    }
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

pub mod server;