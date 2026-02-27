use embedded_hal::digital::v2::OutputPin; // RS485 DE pin output control.
use embedded_hal::serial::{Read as SerialRead, Write as SerialWrite}; // UART RX/TX trait aliases.

pub const ECP_ADDR: u8 = 0x01; // Fixed slave address for this controller.
const ECP_DELIM: u8 = 0xFF; // Frame delimiter byte.
const ECP_OVERHEAD_MIN: usize = 6; // len + addr + act + crc_hi + crc_lo + delim.
const ECP_LEN_IDX: usize = 0; // Length field index.
const ECP_ADDR_IDX: usize = 1; // Address field index.
const ECP_ACT_IDX: usize = 2; // Action field index.
const ECP_PAYLOAD_IDX: usize = 3; // Payload start index.

const ECP_ACT_INIT: u8 = 0x00; // Init frame action.
const ECP_ACT_REQ_SEND: u8 = 0x01; // Master poll action.
const ECP_ACT_SEND_NOTIFY: u8 = 0x02; // Update-count announce action.
const ECP_ACT_GET_PURPOSE: u8 = 0x10; // Master capability query that completes slave init in reference implementation.
const ECP_ACT_SPECIAL_INTERACT: u8 = 0x11; // Special-device interaction action (docs-compatible).

const ECP_SPECIALDEV_IBUTTON: u8 = 0x07; // iButton special-device id.
const ECP_IBUTTON_GET_DEV_CNT: u8 = 0x00; // iButton sub-action: get reader count.
const ECP_IBUTTON_GET_TAG: u8 = 0x01; // iButton sub-action: get tag by index.

pub struct EcprotoState {
    rx_buf: [u8; 256], // Incoming frame assembly buffer.
    rx_len: usize, // Number of bytes currently buffered.
    initialized: bool, // Whether init frame has been sent.
}

impl EcprotoState {
    pub fn new() -> Self {
        Self {
            rx_buf: [0u8; 256], // Zeroed receive buffer.
            rx_len: 0, // Empty RX cursor.
            initialized: false, // Start uninitialized and wait for master INIT handshake.
        }
    }
}

fn ecp_crc16_ibm(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16; // CRC16-IBM initial value.
    for &b in data {
        crc ^= b as u16; // XOR byte into low CRC byte.
        for _ in 0..8 {
            if (crc & 0x0001) != 0 {
                crc >>= 1; // Shift right.
                crc ^= 0xA001; // Apply reflected polynomial.
            } else {
                crc >>= 1; // Shift right with no polynomial.
            }
        }
    }
    crc // Final CRC16 value.
}

fn ecp_send_frame<TX, DE>(tx: &mut TX, de: &mut DE, action: u8, payload: &[u8])
where
    TX: SerialWrite<u8>,
    DE: OutputPin,
{
    let mut frame = [0u8; 256]; // Local TX frame buffer.
    let mut len = ECP_OVERHEAD_MIN + payload.len(); // Initial length including trailer overhead.
    if len > 255 {
        return; // Length field is one byte; reject oversized frame.
    }

    frame[ECP_LEN_IDX] = len as u8; // Write frame length.
    frame[ECP_ADDR_IDX] = ECP_ADDR; // Write destination/source address.
    frame[ECP_ACT_IDX] = action; // Write action id.
    for (i, b) in payload.iter().enumerate() {
        frame[ECP_PAYLOAD_IDX + i] = *b; // Copy payload byte-by-byte.
    }

    loop {
        let crc = ecp_crc16_ibm(&frame[..len - 3]); // CRC over bytes before CRC+delimiter.
        let crc_hi = ((crc >> 8) & 0xFF) as u8; // High CRC byte.
        let crc_lo = (crc & 0xFF) as u8; // Low CRC byte.
        if crc_hi == ECP_DELIM || crc_lo == ECP_DELIM {
            if len >= 255 {
                return; // Cannot pad further without overflowing length field.
            }
            len += 1; // Add zero padding byte before CRC.
            frame[ECP_LEN_IDX] = len as u8; // Update frame length after padding.
            continue; // Recompute CRC for new length.
        }
        frame[len - 3] = crc_hi; // Store CRC high byte.
        frame[len - 2] = crc_lo; // Store CRC low byte.
        frame[len - 1] = ECP_DELIM; // Store frame delimiter.
        break; // CRC bytes valid, frame complete.
    }

    let _ = de.set_high(); // RS485: enable transmit mode.
    for b in &frame[..len] {
        loop {
            if tx.write(*b).is_ok() {
                break; // Send byte when UART accepts it.
            }
        }
    }
    loop {
        if tx.flush().is_ok() {
            break; // Wait until final byte fully shifted out.
        }
    }
    let _ = de.set_low(); // RS485: return to receive mode.
}

fn ecp_send_ibutton_tag<TX, DE>(tx: &mut TX, de: &mut DE, dev_id: u8, present: bool, rom: &[u8; 8])
where
    TX: SerialWrite<u8>,
    DE: OutputPin,
{
    let mut payload = [0u8; 12]; // iButton payload format used by ecproto C code.
    payload[0] = ECP_SPECIALDEV_IBUTTON; // Special-device selector.
    payload[1] = ECP_IBUTTON_GET_TAG; // Sub-action selector.
    payload[2] = dev_id; // Reader index.
    payload[3] = if present { 1 } else { 0 }; // Presence flag.
    if present {
        payload[4] = rom[0]; // Family code.
        payload[5..11].copy_from_slice(&rom[1..7]); // 6 UID bytes.
        payload[11] = rom[7]; // ROM CRC byte.
    }
    ecp_send_frame(tx, de, ECP_ACT_SPECIAL_INTERACT, &payload); // Send fully encoded frame.
}

fn ecp_process_updates<TX, DE>(
    tx: &mut TX, // UART TX handle.
    de: &mut DE, // RS485 direction pin.
    state: &mut EcprotoState, // Protocol state.
    dirty: &mut [bool; 8], // Per-reader changed flags.
    present: &[bool; 8], // Per-reader presence cache.
    uids: &[[u8; 8]; 8], // Per-reader UID cache.
) where
    TX: SerialWrite<u8>,
    DE: OutputPin,
{
    if !state.initialized {
        ecp_send_frame(tx, de, ECP_ACT_INIT, &[]); // Request master init per protocol.
        return; // Do not send updates until INIT is received from master.
    }

    let mut update_cnt = 0u8; // Count how many updates we will emit.
    for d in dirty.iter() {
        if *d {
            update_cnt = update_cnt.saturating_add(1); // Count changed readers.
        }
    }

    ecp_send_frame(tx, de, ECP_ACT_SEND_NOTIFY, &[update_cnt]); // First frame: number of updates.
    for i in 0..8 {
        if dirty[i] {
            ecp_send_ibutton_tag(tx, de, i as u8, present[i], &uids[i]); // Send one update frame.
            dirty[i] = false; // Clear dirty flag after sending.
        }
    }
}

fn ecp_handle_frame<TX, DE, REFRESH>(
    tx: &mut TX, // UART TX.
    de: &mut DE, // RS485 direction pin.
    state: &mut EcprotoState, // Protocol state.
    frame: &[u8], // Complete received frame bytes.
    dirty: &mut [bool; 8], // Dirty flags for update generation.
    present: &mut [bool; 8], // Presence cache.
    uids: &mut [[u8; 8]; 8], // UID cache.
    refresh: &mut REFRESH, // On-demand scan callback (single reader or full sweep).
) where
    TX: SerialWrite<u8>,
    DE: OutputPin,
    REFRESH: FnMut(Option<usize>, &mut [bool; 8], &mut [[u8; 8]; 8], &mut [bool; 8]),
{
    if frame.len() < ECP_OVERHEAD_MIN {
        return; // Reject too-short frame.
    }
    let declared_len = frame[ECP_LEN_IDX] as usize; // Length value from header.
    if declared_len != frame.len() {
        return; // Reject length mismatch.
    }
    if frame[declared_len - 1] != ECP_DELIM {
        return; // Reject missing delimiter.
    }
    let crc_is = ((frame[declared_len - 3] as u16) << 8) | frame[declared_len - 2] as u16; // CRC from frame.
    let crc_should = ecp_crc16_ibm(&frame[..declared_len - 3]); // Recomputed CRC.
    if crc_is != crc_should {
        return; // Reject bad CRC.
    }
    if frame[ECP_ADDR_IDX] != ECP_ADDR {
        return; // Ignore frames for other addresses.
    }

    match frame[ECP_ACT_IDX] {
        ECP_ACT_INIT => {
            state.initialized = true; // Compatibility path for masters that still use INIT as handshake.
        }
        ECP_ACT_GET_PURPOSE => {
            state.initialized = true; // Reference path: master GET_PURPOSE marks slave initialized.
        }
        ECP_ACT_REQ_SEND => {
            if state.initialized {
                refresh(None, present, uids, dirty); // Poll request updates all readers first.
            }
            ecp_process_updates(tx, de, state, dirty, present, uids); // Poll request -> send updates.
        }
        ECP_ACT_SPECIAL_INTERACT => {
            let payload = &frame[ECP_PAYLOAD_IDX..declared_len - 3]; // Strip header/trailer.
            if payload.len() < 2 {
                return; // Need at least device + sub-action.
            }
            if payload[0] != ECP_SPECIALDEV_IBUTTON {
                return; // Only iButton implemented currently.
            }
            match payload[1] {
                ECP_IBUTTON_GET_DEV_CNT => {
                    let out = [ECP_SPECIALDEV_IBUTTON, ECP_IBUTTON_GET_DEV_CNT, 8u8]; // Report 8 readers.
                    ecp_send_frame(tx, de, ECP_ACT_SPECIAL_INTERACT, &out); // Reply frame.
                }
                ECP_IBUTTON_GET_TAG => {
                    if payload.len() < 3 {
                        return; // Need requested reader index.
                    }
                    let idx = payload[2] as usize; // Parse reader index.
                    if idx >= 8 {
                        return; // Reject out-of-range index.
                    }
                    refresh(Some(idx), present, uids, dirty); // Refresh requested reader just-in-time.
                    ecp_send_ibutton_tag(tx, de, idx as u8, present[idx], &uids[idx]); // Send cached tag state.
                }
                _ => {} // Ignore unsupported iButton sub-actions.
            }
        }
        _ => {} // Ignore unsupported actions.
    }
}

pub fn poll_uart<TX, RX, DE, REFRESH>(
    tx: &mut TX, // UART TX handle.
    rx: &mut RX, // UART RX handle.
    de: &mut DE, // RS485 DE/RE pin.
    state: &mut EcprotoState, // Protocol parser state.
    dirty: &mut [bool; 8], // Dirty flags.
    present: &mut [bool; 8], // Presence cache.
    uids: &mut [[u8; 8]; 8], // UID cache.
    refresh: &mut REFRESH, // On-demand scan callback (single reader or full sweep).
) where
    TX: SerialWrite<u8>,
    RX: SerialRead<u8>,
    DE: OutputPin,
    REFRESH: FnMut(Option<usize>, &mut [bool; 8], &mut [[u8; 8]; 8], &mut [bool; 8]),
{
    loop {
        let byte = match rx.read() {
            Ok(b) => b, // Read one byte from UART.
            Err(_) => break, // Stop when RX FIFO is empty.
        };

        if state.rx_len >= state.rx_buf.len() {
            state.rx_len = 0; // Reset parser if buffer overflow would occur.
        }
        state.rx_buf[state.rx_len] = byte; // Store byte into parser buffer.
        state.rx_len += 1; // Advance parser cursor.

        if state.rx_len >= 3 {
            let declared_len = state.rx_buf[ECP_LEN_IDX] as usize; // Read expected frame length.
            if byte == ECP_DELIM && declared_len <= state.rx_len {
                let mut frame = [0u8; 256]; // Temp frame copy for isolated parse.
                let frame_len = state.rx_len; // Current buffered frame length.
                frame[..frame_len].copy_from_slice(&state.rx_buf[..frame_len]); // Copy bytes.
                state.rx_len = 0; // Reset buffer for next frame.
                ecp_handle_frame(tx, de, state, &frame[..frame_len], dirty, present, uids, refresh); // Parse and respond.
            }
        }
    }
}
