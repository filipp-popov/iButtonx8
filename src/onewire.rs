use embedded_hal::digital::v2::{InputPin, OutputPin}; // GPIO traits used by 1-Wire bus operations.

pub fn delay_us(us: u32, cycles_per_us: u32) {
    cortex_m::asm::delay(us.saturating_mul(cycles_per_us)); // Busy-wait for requested microseconds.
}

fn delay_ms_coop<F>(ms: u32, cycles_per_us: u32, service: &mut F)
where
    F: FnMut(),
{
    for _ in 0..ms {
        delay_us(1_000, cycles_per_us); // Keep 1 ms granularity for predictable timing.
        service(); // Give communication/task layer a chance to run between delay slices.
    }
}

fn ow_reset<P>(pin: &mut P, cycles_per_us: u32) -> bool
where
    P: OutputPin + InputPin, // 1-Wire line must be both driven and sampled.
{
    let _ = pin.set_low(); // Pull line low to start reset pulse.
    delay_us(480, cycles_per_us); // 1-Wire reset low time.
    let _ = pin.set_high(); // Release line (open-drain high via pull-up).
    delay_us(70, cycles_per_us); // Wait before presence sample.
    let present = pin.is_low().unwrap_or(false); // Presence pulse = line held low by device.
    delay_us(410, cycles_per_us); // Complete reset timeslot.
    present // True if at least one device responded.
}

fn ow_write_bit<P>(pin: &mut P, bit: bool, cycles_per_us: u32)
where
    P: OutputPin + InputPin,
{
    let _ = pin.set_low(); // Start write slot with low pulse.
    if bit {
        delay_us(6, cycles_per_us); // Short low for writing logic 1.
        let _ = pin.set_high(); // Release quickly.
        delay_us(64, cycles_per_us); // Finish slot timing.
    } else {
        delay_us(60, cycles_per_us); // Long low for writing logic 0.
        let _ = pin.set_high(); // Release near slot end.
        delay_us(10, cycles_per_us); // Recovery time.
    }
}

fn ow_read_bit<P>(pin: &mut P, cycles_per_us: u32) -> bool
where
    P: OutputPin + InputPin,
{
    let _ = pin.set_low(); // Initiate read slot.
    delay_us(6, cycles_per_us); // Keep low briefly.
    let _ = pin.set_high(); // Release so slave can drive bit value.
    delay_us(9, cycles_per_us); // Wait to sample window.
    let bit = pin.is_high().unwrap_or(false); // Sample line logic level.
    delay_us(55, cycles_per_us); // Finish slot/recovery.
    bit // Return sampled bit.
}

fn ow_write_byte<P>(pin: &mut P, mut b: u8, cycles_per_us: u32)
where
    P: OutputPin + InputPin,
{
    for _ in 0..8 { // Send 8 bits LSB-first.
        ow_write_bit(pin, (b & 0x01) != 0, cycles_per_us); // Write least significant bit.
        b >>= 1; // Shift next bit into LSB position.
    }
}

fn ow_read_byte<P>(pin: &mut P, cycles_per_us: u32) -> u8
where
    P: OutputPin + InputPin,
{
    let mut b = 0u8; // Byte accumulator.
    for i in 0..8 { // Read 8 bits LSB-first.
        if ow_read_bit(pin, cycles_per_us) {
            b |= 1 << i; // Set bit i if sampled high.
        }
    }
    b // Return assembled byte.
}

fn ow_crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8; // Dallas/Maxim CRC8 state.
    for &byte in data {
        let mut inb = byte; // Shift register copy of current input byte.
        for _ in 0..8 {
            let mix = (crc ^ inb) & 0x01; // Check if polynomial step needed.
            crc >>= 1; // Shift CRC right.
            if mix != 0 {
                crc ^= 0x8C; // Apply reflected CRC8 polynomial.
            }
            inb >>= 1; // Shift to next input bit.
        }
    }
    crc // Return final CRC value.
}

fn ibutton_read_rom<P>(pin: &mut P, cycles_per_us: u32) -> Option<[u8; 8]>
where
    P: OutputPin + InputPin,
{
    if !ow_reset(pin, cycles_per_us) {
        return None; // No tag/device present on this line.
    }

    ow_write_byte(pin, 0x33, cycles_per_us); // READ ROM command for single-drop bus.

    let mut rom = [0u8; 8]; // [family][serial x6][crc].
    for byte in &mut rom {
        *byte = ow_read_byte(pin, cycles_per_us); // Read one byte at a time.
    }

    if ow_crc8(&rom[..7]) == rom[7] {
        Some(rom) // CRC valid -> successful ROM read.
    } else {
        None // CRC failed -> reject read.
    }
}

pub fn scan_reader<PWR, DATA>(
    pwr: &mut PWR, // Per-reader power control pin.
    data: &mut DATA, // Per-reader 1-Wire data pin.
    uid_out: &mut [u8; 8], // Output buffer for last valid ROM code.
    cycles_per_us: u32, // Timing conversion factor from clocks.
    service: &mut impl FnMut(), // Cooperative callback (e.g., service RS485).
) -> bool
where
    PWR: OutputPin,
    DATA: OutputPin + InputPin,
{
    let _ = pwr.set_high(); // Power reader ON.
    delay_ms_coop(100, cycles_per_us, service); // Settle delay while still servicing communication.

    let mut found = None; // Store a valid ROM if read succeeds.
    for _ in 0..2 { // Small retry count for robustness.
        if let Some(rom) = ibutton_read_rom(data, cycles_per_us) {
            found = Some(rom); // Save successful UID.
            break; // Stop retries after first success.
        }
        delay_ms_coop(5, cycles_per_us, service); // Retry gap with cooperative servicing.
    }

    let _ = pwr.set_low(); // Power reader OFF before switching to next channel.
    delay_ms_coop(10, cycles_per_us, service); // Guard delay with cooperative servicing.

    if let Some(rom) = found {
        *uid_out = rom; // Update caller cache with valid UID.
        true // Report successful read.
    } else {
        false // Report no valid tag read.
    }
}
