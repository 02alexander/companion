use assign_resources::assign_resources;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use core::sync::atomic::{self, AtomicU32};
use cyw43::{Control, JoinOptions};
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_net::tcp::TcpSocket;
use embassy_rp::{bind_interrupts, pwm, Peripherals};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::interrupt::typelevel::Interrupt;
use embassy_rp::pio::Pio;
use embassy_time::{Duration, Timer};
use rand::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use crate::Irqs;
use crate::assign::*;

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}


pub async fn start_network(net: Netresources, spawner: &Spawner) -> (embassy_net::Stack<'static>, Control<'static>) {

    let pwr = Output::new(net.pwr, Level::Low);
    let cs = Output::new(net.cs, Level::High);
    let mut pio = Pio::new(net.pio, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        net.dio,
        net.spi_clk,
        net.dma,
    );
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let cyw_state = STATE.init(cyw43::State::new());


    let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };
    
    let (net_device, mut control, runner) = cyw43::new(cyw_state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    unsafe { embassy_rp::interrupt::typelevel::IO_IRQ_BANK0_NS::enable() };

    let mut rng = RoscRng;
    let seed = rng.next_u64();


    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let network_config = embassy_net::Config::dhcpv4(Default::default());
    let (stack, runner) = embassy_net::new(
        net_device,
        network_config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    unwrap!(spawner.spawn(net_task(runner)));

    (stack, control)

}


#[embassy_executor::task]
pub async fn transmitter(stack: embassy_net::Stack<'static>, mut control: Control<'static>) {
    loop {
        match control
            .join(
                "Berntsson2.4GHz",
                JoinOptions::new("5826hej5826".as_bytes()),
            )
            .await
        {
            Ok(_) => {
                info!("Joined network!");
                break;
            }
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    let mut wait_time = 1;
    loop {
        if let Some(config_v4) = stack.config_v4() {
            let address = config_v4.address.address().octets();
            info!("{}", address);
            break;
        }
        Timer::after_millis(500*wait_time).await;
        wait_time = (wait_time*2).min(8);
        warn!("Failed to configure network, trying again...");
    }

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];
    
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        control.gpio_set(0, false).await;
        info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            warn!("accept error: {:?}", e);
            continue;
        }

        info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;
        Timer::after_millis(100).await;

        // loop {
        //     let n = match socket.read(&mut buf).await {
        //         Ok(0) => {
        //             warn!("read EOF");
        //             break;
        //         }
        //         Ok(n) => n,
        //         Err(e) => {
        //             warn!("read error: {:?}", e);
        //             break;
        //         }
        //     };


        //     match socket.write_all(&buf[..n]).await {
        //         Ok(()) => {}
        //         Err(e) => {
        //             warn!("write error: {:?}", e);
        //             break;
        //         }
        //     };
        // }
    }
}
