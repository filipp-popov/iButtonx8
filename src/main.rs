#![no_std] // No standard library on MCU targets.
#![no_main] // Use cortex-m runtime entry instead of normal host runtime.

mod ecproto; // RS485 + ECPROTO parser/serializer module.
mod onewire; // 1-Wire low-level timing + reader scan module.

use panic_halt as _; // Halt CPU on panic (smallest panic behavior).

use cortex_m_rt::entry; // Entry attribute for embedded start function.
use stm32f1xx_hal::serial::{Config, Serial}; // UART config/types.
use stm32f1xx_hal::{pac, prelude::*}; // PAC peripherals + extension traits.

use crate::ecproto::{poll_uart, EcprotoState}; // ECPROTO RX/TX polling and state.
use crate::onewire::scan_reader; // Per-reader scan helper.

#[entry] // Mark this as the startup function.
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap(); // Take singleton peripheral set.

    let mut flash = dp.FLASH.constrain(); // FLASH interface wrapper for latency setup.
    let rcc = dp.RCC.constrain(); // RCC wrapper for clock tree configuration.
    let mut afio = dp.AFIO.constrain(); // AFIO wrapper for remap/debug pin control.

    let clocks = rcc // Start clock config builder.
        .cfgr // Access CFGR configuration builder.
        .use_hse(8.MHz()) // Use external 8 MHz crystal input.
        .sysclk(72.MHz()) // Run core/system clock at 72 MHz.
        .pclk1(36.MHz()) // Keep APB1 within STM32F1 limit.
        .freeze(&mut flash.acr); // Apply config and compute frozen clock values.
    let cycles_per_us = clocks.sysclk().raw() / 1_000_000; // Convert microseconds to CPU cycles.

    let mut gpioa = dp.GPIOA.split(); // Split GPIOA into independent pin objects.
    let mut gpiob = dp.GPIOB.split(); // Split GPIOB into independent pin objects.

    // Disable JTAG and keep SWD so PA15/PB3/PB4 become usable GPIOs.
    let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // Reader power control pins: PA0..PA7 (transistor base drive, active high).
    let mut p0 = gpioa.pa0.into_push_pull_output(&mut gpioa.crl); // Reader 0 power.
    let mut p1 = gpioa.pa1.into_push_pull_output(&mut gpioa.crl); // Reader 1 power.
    let mut p2 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl); // Reader 2 power.
    let mut p3 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl); // Reader 3 power.
    let mut p4 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl); // Reader 4 power.
    let mut p5 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl); // Reader 5 power.
    let mut p6 = gpioa.pa6.into_push_pull_output(&mut gpioa.crl); // Reader 6 power.
    let mut p7 = gpioa.pa7.into_push_pull_output(&mut gpioa.crl); // Reader 7 power.
    let _ = p0.set_low(); // Ensure reader 0 starts powered off.
    let _ = p1.set_low(); // Ensure reader 1 starts powered off.
    let _ = p2.set_low(); // Ensure reader 2 starts powered off.
    let _ = p3.set_low(); // Ensure reader 3 starts powered off.
    let _ = p4.set_low(); // Ensure reader 4 starts powered off.
    let _ = p5.set_low(); // Ensure reader 5 starts powered off.
    let _ = p6.set_low(); // Ensure reader 6 starts powered off.
    let _ = p7.set_low(); // Ensure reader 7 starts powered off.

    // Reader data pins as open-drain outputs (release high via external pull-up).
    let mut d0 = gpiob.pb9.into_open_drain_output(&mut gpiob.crh); // Reader 0 data.
    let mut d1 = gpiob.pb8.into_open_drain_output(&mut gpiob.crh); // Reader 1 data.
    let mut d2 = gpiob.pb7.into_open_drain_output(&mut gpiob.crl); // Reader 2 data.
    let mut d3 = gpiob.pb6.into_open_drain_output(&mut gpiob.crl); // Reader 3 data.
    let mut d4 = gpiob.pb5.into_open_drain_output(&mut gpiob.crl); // Reader 4 data.
    let mut d5 = pb4.into_open_drain_output(&mut gpiob.crl); // Reader 5 data.
    let mut d6 = pb3.into_open_drain_output(&mut gpiob.crl); // Reader 6 data.
    let mut d7 = pa15.into_open_drain_output(&mut gpioa.crh); // Reader 7 data.
    let _ = d0.set_high(); // Release line to pulled-up idle state.
    let _ = d1.set_high(); // Release line to pulled-up idle state.
    let _ = d2.set_high(); // Release line to pulled-up idle state.
    let _ = d3.set_high(); // Release line to pulled-up idle state.
    let _ = d4.set_high(); // Release line to pulled-up idle state.
    let _ = d5.set_high(); // Release line to pulled-up idle state.
    let _ = d6.set_high(); // Release line to pulled-up idle state.
    let _ = d7.set_high(); // Release line to pulled-up idle state.

    // UART1 wiring for RS485 bus.
    let tx1 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh); // USART1 TX pin.
    let rx1 = gpioa.pa10; // USART1 RX pin.
    let mut rs485_de = gpioa.pa8.into_push_pull_output(&mut gpioa.crh); // DE/RE direction pin.
    let _ = rs485_de.set_low(); // Low = receive mode by default.

    let serial = Serial::new(
        dp.USART1, // UART peripheral instance.
        (tx1, rx1), // UART pin tuple.
        &mut afio.mapr, // AFIO remap register access.
        Config::default().baudrate(38_400.bps()), // UART config: 38400 8N1.
        &clocks, // Clock frequencies for baud generation.
    );
    let (mut uart_tx, mut uart_rx) = serial.split(); // Split into independent TX/RX handles.

    // Presence LEDs for readers 0..7.
    let mut l0 = gpiob.pb0.into_push_pull_output(&mut gpiob.crl); // LED0.
    let mut l1 = gpiob.pb1.into_push_pull_output(&mut gpiob.crl); // LED1.
    let mut l2 = gpiob.pb10.into_push_pull_output(&mut gpiob.crh); // LED2.
    let mut l3 = gpiob.pb11.into_push_pull_output(&mut gpiob.crh); // LED3.
    let mut l4 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh); // LED4.
    let mut l5 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh); // LED5.
    let mut l6 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh); // LED6.
    let mut l7 = gpiob.pb15.into_push_pull_output(&mut gpiob.crh); // LED7.
    let _ = l0.set_low(); // Start LED0 off.
    let _ = l1.set_low(); // Start LED1 off.
    let _ = l2.set_low(); // Start LED2 off.
    let _ = l3.set_low(); // Start LED3 off.
    let _ = l4.set_low(); // Start LED4 off.
    let _ = l5.set_low(); // Start LED5 off.
    let _ = l6.set_low(); // Start LED6 off.
    let _ = l7.set_low(); // Start LED7 off.

    let mut uids = [[0u8; 8]; 8]; // Last valid UID per reader.
    let mut present = [false; 8]; // Presence state per reader.
    let mut dirty = [true; 8]; // Change flag per reader for ECPROTO update flow.
    let mut ecp = EcprotoState::new(); // ECPROTO parser/buffer state.
    let mut scan_idx = 0usize; // Round-robin scanner index 0..7.

    loop {
        // Service UART RX/TX before scanning to keep protocol latency low.
        poll_uart(
            &mut uart_tx,
            &mut uart_rx,
            &mut rs485_de,
            &mut ecp,
            &mut dirty,
            &present,
            &uids,
        );

        let old_present = present[scan_idx]; // Snapshot previous presence.
        let old_uid = uids[scan_idx]; // Snapshot previous UID.

        // Scan exactly one reader per loop iteration.
        match scan_idx {
            0 => present[0] = scan_reader(&mut p0, &mut d0, &mut uids[0], cycles_per_us), // Reader 0.
            1 => present[1] = scan_reader(&mut p1, &mut d1, &mut uids[1], cycles_per_us), // Reader 1.
            2 => present[2] = scan_reader(&mut p2, &mut d2, &mut uids[2], cycles_per_us), // Reader 2.
            3 => present[3] = scan_reader(&mut p3, &mut d3, &mut uids[3], cycles_per_us), // Reader 3.
            4 => present[4] = scan_reader(&mut p4, &mut d4, &mut uids[4], cycles_per_us), // Reader 4.
            5 => present[5] = scan_reader(&mut p5, &mut d5, &mut uids[5], cycles_per_us), // Reader 5.
            6 => present[6] = scan_reader(&mut p6, &mut d6, &mut uids[6], cycles_per_us), // Reader 6.
            _ => present[7] = scan_reader(&mut p7, &mut d7, &mut uids[7], cycles_per_us), // Reader 7.
        }

        // Mark reader dirty if presence or UID changed.
        if present[scan_idx] != old_present || uids[scan_idx] != old_uid {
            dirty[scan_idx] = true;
        }
        scan_idx = (scan_idx + 1) & 0x07; // Advance and wrap index to [0..7].

        // Service UART again after scan so requests are handled quickly.
        poll_uart(
            &mut uart_tx,
            &mut uart_rx,
            &mut rs485_de,
            &mut ecp,
            &mut dirty,
            &present,
            &uids,
        );

        // Mirror presence state to LED outputs.
        if present[0] { let _ = l0.set_high(); } else { let _ = l0.set_low(); } // LED0 status.
        if present[1] { let _ = l1.set_high(); } else { let _ = l1.set_low(); } // LED1 status.
        if present[2] { let _ = l2.set_high(); } else { let _ = l2.set_low(); } // LED2 status.
        if present[3] { let _ = l3.set_high(); } else { let _ = l3.set_low(); } // LED3 status.
        if present[4] { let _ = l4.set_high(); } else { let _ = l4.set_low(); } // LED4 status.
        if present[5] { let _ = l5.set_high(); } else { let _ = l5.set_low(); } // LED5 status.
        if present[6] { let _ = l6.set_high(); } else { let _ = l6.set_low(); } // LED6 status.
        if present[7] { let _ = l7.set_high(); } else { let _ = l7.set_low(); } // LED7 status.
    }
}
