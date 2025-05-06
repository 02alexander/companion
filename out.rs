#![feature(prelude_import)]
//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]
#[prelude_import]
use core::prelude::rust_2024::*;
#[macro_use]
extern crate core;
extern crate compiler_builtins as _;

use core::sync::atomic::{self, AtomicU32};

use cyw43::JoinOptions;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_net::tcp::TcpSocket;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::interrupt::typelevel::Interrupt;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use rand::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    // let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    // let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    // probe-rs download ../../cyw43-firmware/43439A0.bin --binary-format bin --chip RP2040 --base-address 0x10100000
    // probe-rs download ../../cyw43-firmware/43439A0_clm.bin --binary-format bin --chip RP2040 --base-address 0x10140000
    //
    // or through usb
    // picotool load -v cyw43-firmware/43439A0.bin -t bin --offset 0x10100000
    // picotool load -v cyw43-firmware/43439A0_clm.bin -t bin --offset 0x10140000
    //
    // After you've flashed the firmware independently, use these lines instead.
    {
        static ENTRY: ::rp_binary_info::StringEntry = ::rp_binary_info::StringEntry::new(
            ::rp_binary_info::consts::TAG_RASPBERRY_PI,
            ::rp_binary_info::consts::ID_RP_PROGRAM_NAME,
            c"Pico template",
        );
        ENTRY.addr()
    },
    {
        static ENTRY: ::rp_binary_info::StringEntry = ::rp_binary_info::StringEntry::new(
            ::rp_binary_info::consts::TAG_RASPBERRY_PI,
            ::rp_binary_info::consts::ID_RP_PROGRAM_DESCRIPTION,
            c"Pico 2W template",
        );
        ENTRY.addr()
    },
    {
        static ENTRY: ::rp_binary_info::StringEntry = ::rp_binary_info::StringEntry::new(
            ::rp_binary_info::consts::TAG_RASPBERRY_PI,
            ::rp_binary_info::consts::ID_RP_PROGRAM_VERSION_STRING,
            {
                let value = "0.1.0\u{0}";
                let value_cstr =
                    unsafe { core::ffi::CStr::from_bytes_with_nul_unchecked(value.as_bytes()) };
                value_cstr
            },
        );
        ENTRY.addr()
    },
    {
        static ENTRY: ::rp_binary_info::StringEntry = ::rp_binary_info::StringEntry::new(
            ::rp_binary_info::consts::TAG_RASPBERRY_PI,
            ::rp_binary_info::consts::ID_RP_PROGRAM_BUILD_ATTRIBUTE,
            {
                if true {
                    c"debug"
                } else {
                    c"release"
                }
            },
        );
        ENTRY.addr()
    },
];
struct Irqs;
#[automatically_derived]
impl ::core::marker::Copy for Irqs {}
#[automatically_derived]
impl ::core::clone::Clone for Irqs {
    #[inline]
    fn clone(&self) -> Irqs {
        *self
    }
}
#[allow(non_snake_case)]
#[no_mangle]
unsafe extern "C" fn PIO0_IRQ_0() {
    <pio::InterruptHandler<PIO0> as ::embassy_rp::interrupt::typelevel::Handler<
        ::embassy_rp::interrupt::typelevel::PIO0_IRQ_0,
    >>::on_interrupt();
}
unsafe impl
    ::embassy_rp::interrupt::typelevel::Binding<
        ::embassy_rp::interrupt::typelevel::PIO0_IRQ_0,
        pio::InterruptHandler<PIO0>,
    > for Irqs
{
}
#[doc(hidden)]
async fn __cyw43_task_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}
fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ::embassy_executor::SpawnToken<impl Sized> {
    const POOL_SIZE: usize = 1;
    static POOL: ::embassy_executor::_export::TaskPoolRef =
        ::embassy_executor::_export::TaskPoolRef::new();
    unsafe {
        POOL.get::<_, POOL_SIZE>()
            ._spawn_async_fn(move || __cyw43_task_task(runner))
    }
}
#[doc(hidden)]
async fn __net_task_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}
fn net_task(
    runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>,
) -> ::embassy_executor::SpawnToken<impl Sized> {
    const POOL_SIZE: usize = 1;
    static POOL: ::embassy_executor::_export::TaskPoolRef =
        ::embassy_executor::_export::TaskPoolRef::new();
    unsafe {
        POOL.get::<_, POOL_SIZE>()
            ._spawn_async_fn(move || __net_task_task(runner))
    }
}
#[doc(hidden)]
#[allow(clippy::future_not_send)]
async fn ____embassy_main_task(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let cyw_state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(cyw_state, pwr, spi, fw).await;
    match defmt::export::into_result(spawner.spawn(cyw43_task(runner))) {
        ::core::result::Result::Ok(res) => res,
        ::core::result::Result::Err(_unwrap_err) => {
            match (&(_unwrap_err)) {
                (arg0) => {
                    if {
                        const CHECK: bool = {
                            const fn check() -> bool {
                                let module_path = "pico2w_template".as_bytes();
                                if if 15usize > module_path.len() {
                                    false
                                } else {
                                    module_path[0usize] == 112u8
                                        && module_path[1usize] == 105u8
                                        && module_path[2usize] == 99u8
                                        && module_path[3usize] == 111u8
                                        && module_path[4usize] == 50u8
                                        && module_path[5usize] == 119u8
                                        && module_path[6usize] == 95u8
                                        && module_path[7usize] == 116u8
                                        && module_path[8usize] == 101u8
                                        && module_path[9usize] == 109u8
                                        && module_path[10usize] == 112u8
                                        && module_path[11usize] == 108u8
                                        && module_path[12usize] == 97u8
                                        && module_path[13usize] == 116u8
                                        && module_path[14usize] == 101u8
                                        && if 15usize == module_path.len() {
                                            true
                                        } else {
                                            module_path[15usize] == b':'
                                        }
                                } {
                                    return true;
                                }
                                false
                            }
                            check()
                        };
                        CHECK
                    } {
                        unsafe {
                            defmt::export::acquire_and_header(&{
                                defmt::export::make_istr({
                                    #[link_section = ".defmt.error.{\"package\":\"pico2w-template\",\"tag\":\"defmt_error\",\"data\":\"panicked at 'unwrap failed: spawner.spawn(cyw43_task(runner))'\\nerror: `{:?}`\",\"disambiguator\":\"15239385218299725453\",\"crate_name\":\"pico2w_template\"}"]
                                    #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_error\",\"data\":\"panicked at 'unwrap failed: spawner.spawn(cyw43_task(runner))'\\nerror: `{:?}`\",\"disambiguator\":\"15239385218299725453\",\"crate_name\":\"pico2w_template\"}"]
                                    static DEFMT_LOG_STATEMENT: u8 = 0;
                                    &DEFMT_LOG_STATEMENT as *const u8 as u16
                                })
                            });
                        };
                        defmt::export::fmt(arg0);
                        unsafe { defmt::export::release() }
                    }
                }
            };
            defmt::export::panic()
        }
    };
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
    match defmt::export::into_result(spawner.spawn(net_task(runner))) {
        ::core::result::Result::Ok(res) => res,
        ::core::result::Result::Err(_unwrap_err) => {
            match (&(_unwrap_err)) {
                (arg0) => {
                    if {
                        const CHECK: bool = {
                            const fn check() -> bool {
                                let module_path = "pico2w_template".as_bytes();
                                if if 15usize > module_path.len() {
                                    false
                                } else {
                                    module_path[0usize] == 112u8
                                        && module_path[1usize] == 105u8
                                        && module_path[2usize] == 99u8
                                        && module_path[3usize] == 111u8
                                        && module_path[4usize] == 50u8
                                        && module_path[5usize] == 119u8
                                        && module_path[6usize] == 95u8
                                        && module_path[7usize] == 116u8
                                        && module_path[8usize] == 101u8
                                        && module_path[9usize] == 109u8
                                        && module_path[10usize] == 112u8
                                        && module_path[11usize] == 108u8
                                        && module_path[12usize] == 97u8
                                        && module_path[13usize] == 116u8
                                        && module_path[14usize] == 101u8
                                        && if 15usize == module_path.len() {
                                            true
                                        } else {
                                            module_path[15usize] == b':'
                                        }
                                } {
                                    return true;
                                }
                                false
                            }
                            check()
                        };
                        CHECK
                    } {
                        unsafe {
                            defmt::export::acquire_and_header(&{
                                defmt::export::make_istr({
                                    #[link_section = ".defmt.error.{\"package\":\"pico2w-template\",\"tag\":\"defmt_error\",\"data\":\"panicked at 'unwrap failed: spawner.spawn(net_task(runner))'\\nerror: `{:?}`\",\"disambiguator\":\"2445160078918498416\",\"crate_name\":\"pico2w_template\"}"]
                                    #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_error\",\"data\":\"panicked at 'unwrap failed: spawner.spawn(net_task(runner))'\\nerror: `{:?}`\",\"disambiguator\":\"2445160078918498416\",\"crate_name\":\"pico2w_template\"}"]
                                    static DEFMT_LOG_STATEMENT: u8 = 0;
                                    &DEFMT_LOG_STATEMENT as *const u8 as u16
                                })
                            });
                        };
                        defmt::export::fmt(arg0);
                        unsafe { defmt::export::release() }
                    }
                }
            };
            defmt::export::panic()
        }
    };
    loop {
        match control
            .join("Berntsson2.4GHz", JoinOptions::new("PASSWORD".as_bytes()))
            .await
        {
            Ok(_) => {
                match () {
                    () => {
                        if {
                            const CHECK: bool = {
                                const fn check() -> bool {
                                    let module_path = "pico2w_template".as_bytes();
                                    if if 15usize > module_path.len() {
                                        false
                                    } else {
                                        module_path[0usize] == 112u8
                                            && module_path[1usize] == 105u8
                                            && module_path[2usize] == 99u8
                                            && module_path[3usize] == 111u8
                                            && module_path[4usize] == 50u8
                                            && module_path[5usize] == 119u8
                                            && module_path[6usize] == 95u8
                                            && module_path[7usize] == 116u8
                                            && module_path[8usize] == 101u8
                                            && module_path[9usize] == 109u8
                                            && module_path[10usize] == 112u8
                                            && module_path[11usize] == 108u8
                                            && module_path[12usize] == 97u8
                                            && module_path[13usize] == 116u8
                                            && module_path[14usize] == 101u8
                                            && if 15usize == module_path.len() {
                                                true
                                            } else {
                                                module_path[15usize] == b':'
                                            }
                                    } {
                                        return true;
                                    }
                                    false
                                }
                                check()
                            };
                            CHECK
                        } {
                            defmt::export::acquire_header_and_release(&{
                                defmt::export::make_istr({
                                    #[link_section = ".defmt.info.{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"Joined network!\",\"disambiguator\":\"512621380199098549\",\"crate_name\":\"pico2w_template\"}"]
                                    #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"Joined network!\",\"disambiguator\":\"512621380199098549\",\"crate_name\":\"pico2w_template\"}"]
                                    static DEFMT_LOG_STATEMENT: u8 = 0;
                                    &DEFMT_LOG_STATEMENT as *const u8 as u16
                                })
                            });
                        }
                    }
                };
                break;
            }
            Err(err) => {
                match (&(err.status)) {
                    (arg0) => {
                        if {
                            const CHECK: bool = {
                                const fn check() -> bool {
                                    let module_path = "pico2w_template".as_bytes();
                                    if if 15usize > module_path.len() {
                                        false
                                    } else {
                                        module_path[0usize] == 112u8
                                            && module_path[1usize] == 105u8
                                            && module_path[2usize] == 99u8
                                            && module_path[3usize] == 111u8
                                            && module_path[4usize] == 50u8
                                            && module_path[5usize] == 119u8
                                            && module_path[6usize] == 95u8
                                            && module_path[7usize] == 116u8
                                            && module_path[8usize] == 101u8
                                            && module_path[9usize] == 109u8
                                            && module_path[10usize] == 112u8
                                            && module_path[11usize] == 108u8
                                            && module_path[12usize] == 97u8
                                            && module_path[13usize] == 116u8
                                            && module_path[14usize] == 101u8
                                            && if 15usize == module_path.len() {
                                                true
                                            } else {
                                                module_path[15usize] == b':'
                                            }
                                    } {
                                        return true;
                                    }
                                    false
                                }
                                check()
                            };
                            CHECK
                        } {
                            unsafe {
                                defmt::export::acquire_and_header(&{
                                    defmt::export::make_istr({
                                        #[link_section = ".defmt.info.{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"join failed with status={}\",\"disambiguator\":\"5657393827728017488\",\"crate_name\":\"pico2w_template\"}"]
                                        #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"join failed with status={}\",\"disambiguator\":\"5657393827728017488\",\"crate_name\":\"pico2w_template\"}"]
                                        static DEFMT_LOG_STATEMENT: u8 = 0;
                                        &DEFMT_LOG_STATEMENT as *const u8 as u16
                                    })
                                });
                            };
                            defmt::export::fmt(arg0);
                            unsafe { defmt::export::release() }
                        }
                    }
                };
            }
        }
    }
    loop {
        if let Some(config_v4) = stack.config_v4() {
            let address = config_v4.address.address().octets();
            match (&(address)) {
                (arg0) => {
                    if {
                        const CHECK: bool = {
                            const fn check() -> bool {
                                let module_path = "pico2w_template".as_bytes();
                                if if 15usize > module_path.len() {
                                    false
                                } else {
                                    module_path[0usize] == 112u8
                                        && module_path[1usize] == 105u8
                                        && module_path[2usize] == 99u8
                                        && module_path[3usize] == 111u8
                                        && module_path[4usize] == 50u8
                                        && module_path[5usize] == 119u8
                                        && module_path[6usize] == 95u8
                                        && module_path[7usize] == 116u8
                                        && module_path[8usize] == 101u8
                                        && module_path[9usize] == 109u8
                                        && module_path[10usize] == 112u8
                                        && module_path[11usize] == 108u8
                                        && module_path[12usize] == 97u8
                                        && module_path[13usize] == 116u8
                                        && module_path[14usize] == 101u8
                                        && if 15usize == module_path.len() {
                                            true
                                        } else {
                                            module_path[15usize] == b':'
                                        }
                                } {
                                    return true;
                                }
                                false
                            }
                            check()
                        };
                        CHECK
                    } {
                        unsafe {
                            defmt::export::acquire_and_header(&{
                                defmt::export::make_istr({
                                    #[link_section = ".defmt.info.{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"{}\",\"disambiguator\":\"7240274185217437555\",\"crate_name\":\"pico2w_template\"}"]
                                    #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"{}\",\"disambiguator\":\"7240274185217437555\",\"crate_name\":\"pico2w_template\"}"]
                                    static DEFMT_LOG_STATEMENT: u8 = 0;
                                    &DEFMT_LOG_STATEMENT as *const u8 as u16
                                })
                            });
                        };
                        defmt::export::fmt(arg0);
                        unsafe { defmt::export::release() }
                    }
                }
            };
            break;
        }
        Timer::after_millis(500).await;
    }
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        control.gpio_set(0, false).await;
        match () {
            () => {
                if {
                    const CHECK: bool = {
                        const fn check() -> bool {
                            let module_path = "pico2w_template".as_bytes();
                            if if 15usize > module_path.len() {
                                false
                            } else {
                                module_path[0usize] == 112u8
                                    && module_path[1usize] == 105u8
                                    && module_path[2usize] == 99u8
                                    && module_path[3usize] == 111u8
                                    && module_path[4usize] == 50u8
                                    && module_path[5usize] == 119u8
                                    && module_path[6usize] == 95u8
                                    && module_path[7usize] == 116u8
                                    && module_path[8usize] == 101u8
                                    && module_path[9usize] == 109u8
                                    && module_path[10usize] == 112u8
                                    && module_path[11usize] == 108u8
                                    && module_path[12usize] == 97u8
                                    && module_path[13usize] == 116u8
                                    && module_path[14usize] == 101u8
                                    && if 15usize == module_path.len() {
                                        true
                                    } else {
                                        module_path[15usize] == b':'
                                    }
                            } {
                                return true;
                            }
                            false
                        }
                        check()
                    };
                    CHECK
                } {
                    defmt::export::acquire_header_and_release(&{
                        defmt::export::make_istr({
                            #[link_section = ".defmt.info.{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"Listening on TCP:1234...\",\"disambiguator\":\"8240278140141475005\",\"crate_name\":\"pico2w_template\"}"]
                            #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"Listening on TCP:1234...\",\"disambiguator\":\"8240278140141475005\",\"crate_name\":\"pico2w_template\"}"]
                            static DEFMT_LOG_STATEMENT: u8 = 0;
                            &DEFMT_LOG_STATEMENT as *const u8 as u16
                        })
                    });
                }
            }
        };
        if let Err(e) = socket.accept(1234).await {
            match (&(e)) {
                (arg0) => {
                    if {
                        const CHECK: bool = {
                            const fn check() -> bool {
                                let module_path = "pico2w_template".as_bytes();
                                if if 15usize > module_path.len() {
                                    false
                                } else {
                                    module_path[0usize] == 112u8
                                        && module_path[1usize] == 105u8
                                        && module_path[2usize] == 99u8
                                        && module_path[3usize] == 111u8
                                        && module_path[4usize] == 50u8
                                        && module_path[5usize] == 119u8
                                        && module_path[6usize] == 95u8
                                        && module_path[7usize] == 116u8
                                        && module_path[8usize] == 101u8
                                        && module_path[9usize] == 109u8
                                        && module_path[10usize] == 112u8
                                        && module_path[11usize] == 108u8
                                        && module_path[12usize] == 97u8
                                        && module_path[13usize] == 116u8
                                        && module_path[14usize] == 101u8
                                        && if 15usize == module_path.len() {
                                            true
                                        } else {
                                            module_path[15usize] == b':'
                                        }
                                } {
                                    return true;
                                }
                                false
                            }
                            check()
                        };
                        CHECK
                    } {
                        unsafe {
                            defmt::export::acquire_and_header(&{
                                defmt::export::make_istr({
                                    #[link_section = ".defmt.warn.{\"package\":\"pico2w-template\",\"tag\":\"defmt_warn\",\"data\":\"accept error: {:?}\",\"disambiguator\":\"406717335064887825\",\"crate_name\":\"pico2w_template\"}"]
                                    #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_warn\",\"data\":\"accept error: {:?}\",\"disambiguator\":\"406717335064887825\",\"crate_name\":\"pico2w_template\"}"]
                                    static DEFMT_LOG_STATEMENT: u8 = 0;
                                    &DEFMT_LOG_STATEMENT as *const u8 as u16
                                })
                            });
                        };
                        defmt::export::fmt(arg0);
                        unsafe { defmt::export::release() }
                    }
                }
            };
            continue;
        }
        match (&(socket.remote_endpoint())) {
            (arg0) => {
                if {
                    const CHECK: bool = {
                        const fn check() -> bool {
                            let module_path = "pico2w_template".as_bytes();
                            if if 15usize > module_path.len() {
                                false
                            } else {
                                module_path[0usize] == 112u8
                                    && module_path[1usize] == 105u8
                                    && module_path[2usize] == 99u8
                                    && module_path[3usize] == 111u8
                                    && module_path[4usize] == 50u8
                                    && module_path[5usize] == 119u8
                                    && module_path[6usize] == 95u8
                                    && module_path[7usize] == 116u8
                                    && module_path[8usize] == 101u8
                                    && module_path[9usize] == 109u8
                                    && module_path[10usize] == 112u8
                                    && module_path[11usize] == 108u8
                                    && module_path[12usize] == 97u8
                                    && module_path[13usize] == 116u8
                                    && module_path[14usize] == 101u8
                                    && if 15usize == module_path.len() {
                                        true
                                    } else {
                                        module_path[15usize] == b':'
                                    }
                            } {
                                return true;
                            }
                            false
                        }
                        check()
                    };
                    CHECK
                } {
                    unsafe {
                        defmt::export::acquire_and_header(&{
                            defmt::export::make_istr({
                                #[link_section = ".defmt.info.{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"Received connection from {:?}\",\"disambiguator\":\"14574805672647616796\",\"crate_name\":\"pico2w_template\"}"]
                                #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_info\",\"data\":\"Received connection from {:?}\",\"disambiguator\":\"14574805672647616796\",\"crate_name\":\"pico2w_template\"}"]
                                static DEFMT_LOG_STATEMENT: u8 = 0;
                                &DEFMT_LOG_STATEMENT as *const u8 as u16
                            })
                        });
                    };
                    defmt::export::fmt(arg0);
                    unsafe { defmt::export::release() }
                }
            }
        };
        control.gpio_set(0, true).await;
        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    match () {
                        () => {
                            if {
                                const CHECK: bool = {
                                    const fn check() -> bool {
                                        let module_path = "pico2w_template".as_bytes();
                                        if if 15usize > module_path.len() {
                                            false
                                        } else {
                                            module_path[0usize] == 112u8
                                                && module_path[1usize] == 105u8
                                                && module_path[2usize] == 99u8
                                                && module_path[3usize] == 111u8
                                                && module_path[4usize] == 50u8
                                                && module_path[5usize] == 119u8
                                                && module_path[6usize] == 95u8
                                                && module_path[7usize] == 116u8
                                                && module_path[8usize] == 101u8
                                                && module_path[9usize] == 109u8
                                                && module_path[10usize] == 112u8
                                                && module_path[11usize] == 108u8
                                                && module_path[12usize] == 97u8
                                                && module_path[13usize] == 116u8
                                                && module_path[14usize] == 101u8
                                                && if 15usize == module_path.len() {
                                                    true
                                                } else {
                                                    module_path[15usize] == b':'
                                                }
                                        } {
                                            return true;
                                        }
                                        false
                                    }
                                    check()
                                };
                                CHECK
                            } {
                                defmt::export::acquire_header_and_release(&{
                                    defmt::export::make_istr({
                                        #[link_section = ".defmt.warn.{\"package\":\"pico2w-template\",\"tag\":\"defmt_warn\",\"data\":\"read EOF\",\"disambiguator\":\"1750585852685124924\",\"crate_name\":\"pico2w_template\"}"]
                                        #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_warn\",\"data\":\"read EOF\",\"disambiguator\":\"1750585852685124924\",\"crate_name\":\"pico2w_template\"}"]
                                        static DEFMT_LOG_STATEMENT: u8 = 0;
                                        &DEFMT_LOG_STATEMENT as *const u8 as u16
                                    })
                                });
                            }
                        }
                    };
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    match (&(e)) {
                        (arg0) => {
                            if {
                                const CHECK: bool = {
                                    const fn check() -> bool {
                                        let module_path = "pico2w_template".as_bytes();
                                        if if 15usize > module_path.len() {
                                            false
                                        } else {
                                            module_path[0usize] == 112u8
                                                && module_path[1usize] == 105u8
                                                && module_path[2usize] == 99u8
                                                && module_path[3usize] == 111u8
                                                && module_path[4usize] == 50u8
                                                && module_path[5usize] == 119u8
                                                && module_path[6usize] == 95u8
                                                && module_path[7usize] == 116u8
                                                && module_path[8usize] == 101u8
                                                && module_path[9usize] == 109u8
                                                && module_path[10usize] == 112u8
                                                && module_path[11usize] == 108u8
                                                && module_path[12usize] == 97u8
                                                && module_path[13usize] == 116u8
                                                && module_path[14usize] == 101u8
                                                && if 15usize == module_path.len() {
                                                    true
                                                } else {
                                                    module_path[15usize] == b':'
                                                }
                                        } {
                                            return true;
                                        }
                                        false
                                    }
                                    check()
                                };
                                CHECK
                            } {
                                unsafe {
                                    defmt::export::acquire_and_header(&{
                                        defmt::export::make_istr({
                                            #[link_section = ".defmt.warn.{\"package\":\"pico2w-template\",\"tag\":\"defmt_warn\",\"data\":\"read error: {:?}\",\"disambiguator\":\"7529443859684305599\",\"crate_name\":\"pico2w_template\"}"]
                                            #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_warn\",\"data\":\"read error: {:?}\",\"disambiguator\":\"7529443859684305599\",\"crate_name\":\"pico2w_template\"}"]
                                            static DEFMT_LOG_STATEMENT: u8 = 0;
                                            &DEFMT_LOG_STATEMENT as *const u8 as u16
                                        })
                                    });
                                };
                                defmt::export::fmt(arg0);
                                unsafe { defmt::export::release() }
                            }
                        }
                    };
                    break;
                }
            };
            match socket.write_all(&buf[..n]).await {
                Ok(()) => {}
                Err(e) => {
                    match (&(e)) {
                        (arg0) => {
                            if {
                                const CHECK: bool = {
                                    const fn check() -> bool {
                                        let module_path = "pico2w_template".as_bytes();
                                        if if 15usize > module_path.len() {
                                            false
                                        } else {
                                            module_path[0usize] == 112u8
                                                && module_path[1usize] == 105u8
                                                && module_path[2usize] == 99u8
                                                && module_path[3usize] == 111u8
                                                && module_path[4usize] == 50u8
                                                && module_path[5usize] == 119u8
                                                && module_path[6usize] == 95u8
                                                && module_path[7usize] == 116u8
                                                && module_path[8usize] == 101u8
                                                && module_path[9usize] == 109u8
                                                && module_path[10usize] == 112u8
                                                && module_path[11usize] == 108u8
                                                && module_path[12usize] == 97u8
                                                && module_path[13usize] == 116u8
                                                && module_path[14usize] == 101u8
                                                && if 15usize == module_path.len() {
                                                    true
                                                } else {
                                                    module_path[15usize] == b':'
                                                }
                                        } {
                                            return true;
                                        }
                                        false
                                    }
                                    check()
                                };
                                CHECK
                            } {
                                unsafe {
                                    defmt::export::acquire_and_header(&{
                                        defmt::export::make_istr({
                                            #[link_section = ".defmt.warn.{\"package\":\"pico2w-template\",\"tag\":\"defmt_warn\",\"data\":\"write error: {:?}\",\"disambiguator\":\"5570638540483538820\",\"crate_name\":\"pico2w_template\"}"]
                                            #[export_name = "{\"package\":\"pico2w-template\",\"tag\":\"defmt_warn\",\"data\":\"write error: {:?}\",\"disambiguator\":\"5570638540483538820\",\"crate_name\":\"pico2w_template\"}"]
                                            static DEFMT_LOG_STATEMENT: u8 = 0;
                                            &DEFMT_LOG_STATEMENT as *const u8 as u16
                                        })
                                    });
                                };
                                defmt::export::fmt(arg0);
                                unsafe { defmt::export::release() }
                            }
                        }
                    };
                    break;
                }
            };
        }
    }
}
#[allow(clippy::future_not_send)]
fn __embassy_main(spawner: Spawner) -> ::embassy_executor::SpawnToken<impl Sized> {
    const POOL_SIZE: usize = 1;
    static POOL: ::embassy_executor::_export::TaskPoolRef =
        ::embassy_executor::_export::TaskPoolRef::new();
    unsafe {
        POOL.get::<_, POOL_SIZE>()
            ._spawn_async_fn(move || ____embassy_main_task(spawner))
    }
}
#[doc(hidden)]
#[export_name = "main"]
pub unsafe extern "C" fn __cortex_m_rt_main_trampoline() {
    #[allow(static_mut_refs)]
    __cortex_m_rt_main()
}
fn __cortex_m_rt_main() -> ! {
    unsafe fn __make_static<T>(t: &mut T) -> &'static mut T {
        ::core::mem::transmute(t)
    }
    let mut executor = ::embassy_executor::Executor::new();
    let executor = unsafe { __make_static(&mut executor) };
    executor.run(|spawner| {
        spawner.must_spawn(__embassy_main(spawner));
    })
}
