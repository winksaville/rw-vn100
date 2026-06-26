//! The `bench` command — configure an output and measure its achieved rate.
//!
//! - `bench_binary` / `bench_ascii` configure, measure, and restore.
//! - `measure_*` count messages/frames; `report_bench` prints the result;
//!   `decode_binary_sample` renders one binary frame.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use crate::cli::ascii_type_name;
use crate::proto::*;
use crate::transact::transact_retry;

// Binary frame: sync 0xFA, groups=0x01 (Common), one 16-bit field mask, the
// selected fields' payload, then a 2-byte CRC. Length depends on the field set.
const BENCH_SYNC: u8 = 0xFA;
const BENCH_GROUPS: u8 = 0x01;
const BENCH_HEADER: usize = 1 + 1 + 2; // sync + groups + field mask
const BENCH_CRC: usize = 2;

/// A measurement outcome: (count, total wire bytes, elapsed seconds, first sample).
type Measured = (u64, u64, f64, Option<String>);

/// Read a little-endian f32 from `buf` at `off`.
fn rd_f32(buf: &[u8], off: usize) -> f32 {
    f32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
}

/// Human-readable decode of the selected fields in one binary frame.
fn decode_binary_sample(frame: &[u8], fields: &[&Field]) -> String {
    let mut off = BENCH_HEADER;
    let mut parts = Vec::new();
    for f in fields {
        let s = match f.name {
            "time" => format!(
                "t={} ns",
                u64::from_le_bytes(frame[off..off + 8].try_into().unwrap())
            ),
            "ypr" => format!(
                "ypr=[{:.2}, {:.2}, {:.2}] deg",
                rd_f32(frame, off),
                rd_f32(frame, off + 4),
                rd_f32(frame, off + 8)
            ),
            "quat" => format!(
                "quat=[{:.4}, {:.4}, {:.4}, {:.4}]",
                rd_f32(frame, off),
                rd_f32(frame, off + 4),
                rd_f32(frame, off + 8),
                rd_f32(frame, off + 12)
            ),
            "gyro" => format!(
                "gyro=[{:.4}, {:.4}, {:.4}] rad/s",
                rd_f32(frame, off),
                rd_f32(frame, off + 4),
                rd_f32(frame, off + 8)
            ),
            "accel" => format!(
                "accel=[{:.3}, {:.3}, {:.3}] m/s^2",
                rd_f32(frame, off),
                rd_f32(frame, off + 4),
                rd_f32(frame, off + 8)
            ),
            "imu" => format!(
                "uncomp_accel=[{:.3}, {:.3}, {:.3}] uncomp_gyro=[{:.4}, {:.4}, {:.4}]",
                rd_f32(frame, off),
                rd_f32(frame, off + 4),
                rd_f32(frame, off + 8),
                rd_f32(frame, off + 12),
                rd_f32(frame, off + 16),
                rd_f32(frame, off + 20)
            ),
            "magpres" => format!(
                "mag=[{:.3}, {:.3}, {:.3}] G temp={:.2} C pres={:.3} kPa",
                rd_f32(frame, off),
                rd_f32(frame, off + 4),
                rd_f32(frame, off + 8),
                rd_f32(frame, off + 12),
                rd_f32(frame, off + 16)
            ),
            _ => "?".to_string(),
        };
        parts.push(s);
        off += f.size;
    }
    parts.join(", ")
}

/// Measure binary frames of `frame_len` bytes for `secs`. Counts CRC-valid
/// frames and captures the first frame's raw bytes for the caller to decode.
fn measure_binary<S: Read>(
    port: &mut S,
    frame_len: usize,
    secs: u64,
) -> std::io::Result<(u64, u64, f64, Option<Vec<u8>>)> {
    let start = Instant::now();
    let deadline = start + Duration::from_secs(secs);
    let mut buf = [0u8; 1024];
    let mut acc: Vec<u8> = Vec::new();
    let mut frames = 0u64;
    let mut first: Option<Vec<u8>> = None;

    while Instant::now() < deadline {
        match port.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => acc.extend_from_slice(&buf[..n]),
            Err(ref e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                ) =>
            {
                continue;
            }
            Err(e) => return Err(e),
        }

        let mut i = 0;
        while i + frame_len <= acc.len() {
            if acc[i] != BENCH_SYNC || acc[i + 1] != BENCH_GROUPS {
                i += 1;
                continue;
            }
            let frame = &acc[i..i + frame_len];
            // CRC over everything after the sync byte (groups..payload..crc) == 0.
            if vn_crc16(&frame[1..]) == 0 {
                frames += 1;
                if first.is_none() {
                    first = Some(frame.to_vec());
                }
                i += frame_len;
            } else {
                i += 1; // false sync (0xFA can appear in payload); resync
            }
        }
        acc.drain(0..i);
        if acc.len() > 8192 {
            let keep = acc.len() - frame_len;
            acc.drain(0..keep);
        }
    }

    Ok((
        frames,
        frames * frame_len as u64,
        start.elapsed().as_secs_f64(),
        first,
    ))
}

/// True if `line` is an async data message (a `$VN...` line that isn't a command
/// echo or error) with a valid checksum.
fn is_async_line(line: &str) -> bool {
    line.starts_with("$VN")
        && !line.starts_with("$VNRRG")
        && !line.starts_with("$VNWRG")
        && !line.starts_with("$VNERR")
        && verify_checksum(line).is_ok()
}

/// Measure ASCII async messages for `secs`: counts valid `$VN...` lines and
/// their wire bytes, capturing the first as the sample.
fn measure_ascii<S: Read>(port: &mut S, secs: u64) -> std::io::Result<Measured> {
    let start = Instant::now();
    let deadline = start + Duration::from_secs(secs);
    let mut buf = [0u8; 1024];
    let mut line: Vec<u8> = Vec::new();
    let mut msgs = 0u64;
    let mut bytes = 0u64;
    let mut sample = None;

    while Instant::now() < deadline {
        let n = match port.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(ref e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                ) =>
            {
                continue;
            }
            Err(e) => return Err(e),
        };
        for &b in &buf[..n] {
            match b {
                b'\n' => {
                    let raw = String::from_utf8_lossy(&line);
                    let cand = match raw.rfind('$') {
                        Some(p) => raw[p..].trim(),
                        None => raw.trim(),
                    };
                    if is_async_line(cand) {
                        msgs += 1;
                        bytes += cand.len() as u64 + 2; // + CRLF
                        if sample.is_none() {
                            sample = Some(cand.to_string());
                        }
                    }
                    line.clear();
                }
                b'\r' => {}
                _ => {
                    line.push(b);
                    if line.len() > 1024 {
                        line.clear();
                    }
                }
            }
        }
    }

    Ok((msgs, bytes, start.elapsed().as_secs_f64(), sample))
}

/// Print the shared bench result block (rate, sample, wire utilization).
fn report_bench(unit: &str, target_hz: u32, m: Measured, baud: u32) {
    let (count, bytes, elapsed, sample) = m;
    let rate = if elapsed > 0.0 {
        count as f64 / elapsed
    } else {
        0.0
    };
    println!("\nResult: {count} {unit} in {elapsed:.2}s = {rate:.1} Hz (target {target_hz} Hz).");
    if let Some(s) = sample {
        println!("Sample: {s}");
    }
    let bits = if elapsed > 0.0 {
        bytes as f64 * 10.0 / elapsed
    } else {
        0.0
    };
    let pct = 100.0 * bits / baud as f64;
    println!(
        "Wire throughput ~{:.0} kbit/s = {:.0}% of the {:.1} kbit/s {baud}-baud link.",
        bits / 1000.0,
        pct,
        baud as f64 / 1000.0
    );
}

/// Configure a binary output (reg 75) with `fields` at `hz`, measure the frame
/// rate for `secs`, then restore the prior state.
///
/// - `serial_port` is the register-75 `asyncMode`: 1 / 2 for one of the
///   VN-100's two UARTs, 3 for both. Binary only streams on the UART the host
///   is wired to, so this must include that port (the RPi5 TTL header is
///   port 2; the RS-232 bench is port 1).
pub fn bench_binary<S: Read + Write>(
    port: &mut S,
    baud: u32,
    hz: u32,
    secs: u64,
    fields: &[&Field],
    serial_port: u8,
) -> Result<(), Box<dyn std::error::Error>> {
    let divisor = 800 / hz; // device IMU base rate is 800 Hz
    let mask: u16 = fields.iter().fold(0, |m, f| m | (1u16 << f.bit));
    let payload: usize = fields.iter().map(|f| f.size).sum();
    let frame_len = BENCH_HEADER + payload + BENCH_CRC;
    let names: Vec<&str> = fields.iter().map(|f| f.name).collect();

    // Remember the current ASCII async rate so we can put it back.
    let prev = transact_retry(
        port,
        &build_command("VNRRG,07"),
        5,
        |l| parse_reg07(l).is_some(),
        "could not read current async rate",
    )?;
    let prev_hz = parse_reg07(&prev).unwrap();

    // Silence the ASCII async output FIRST, before configuring the binary
    // output. The device's fit check (message_size x rate <= baud) runs on the
    // reg-75 write against the SUM of the streams on that port, so leaving ASCII
    // async running can make the binary config fail with $VNERR 0x0C even when
    // the binary stream fits the link on its own. Silencing first also means we
    // measure ONLY the binary stream.
    transact_retry(
        port,
        &build_command("VNWRG,07,0"),
        5,
        |l| parse_reg07(l).is_some(),
        "could not disable ASCII async output",
    )?;

    // Configure Binary Output 1 (reg 75) on the chosen serial port(s). A $VNERR
    // here now means the binary stream alone won't fit the current baud. If the
    // write fails, restore the ASCII async rate before bailing so we don't leave
    // the device with async output switched off.
    let cfg = format!("VNWRG,75,{serial_port},{divisor},01,{mask:04X}");
    if let Err(e) = transact_retry(
        port,
        &build_command(&cfg),
        5,
        |l| l.starts_with("$VNWRG,75"),
        "device did not accept the binary config (a $VNERR means it won't fit at this baud)",
    ) {
        let _ = transact_retry(
            port,
            &build_command(&format!("VNWRG,07,{prev_hz}")),
            3,
            |l| parse_reg07(l).is_some(),
            "restore: ASCII async rate",
        );
        return Err(e);
    }
    println!(
        "Configured binary output: Common{names:?} @ {hz} Hz (divisor {divisor}, {frame_len} B/frame)."
    );

    println!("Measuring for {secs}s...");
    let (frames, bytes, elapsed, first) = measure_binary(port, frame_len, secs)?;
    let sample = first.map(|fr| decode_binary_sample(&fr, fields));
    report_bench("frames", hz, (frames, bytes, elapsed, sample), baud);

    // Restore: turn the binary output off, put the ASCII rate back.
    let _ = transact_retry(
        port,
        &build_command(&format!("VNWRG,75,0,{divisor},01,{mask:04X}")),
        3,
        |l| l.starts_with("$VNWRG,75"),
        "restore: disable binary output",
    );
    let _ = transact_retry(
        port,
        &build_command(&format!("VNWRG,07,{prev_hz}")),
        3,
        |l| parse_reg07(l).is_some(),
        "restore: ASCII async rate",
    );
    println!("Restored: binary output off, ASCII async back to {prev_hz} Hz.");

    if frames == 0 {
        return Err(
            "received 0 binary frames — the config may have targeted the wrong serial port \
             (try a VN-100 on serial1), or the device isn't streaming"
                .into(),
        );
    }
    Ok(())
}

/// Set the ASCII async rate (reg 7) to `hz` — and optionally the message type
/// (reg 6) — measure the message rate for `secs`, then restore prior state.
pub fn bench_ascii<S: Read + Write>(
    port: &mut S,
    baud: u32,
    hz: u32,
    secs: u64,
    ascii_type: Option<u8>,
) -> Result<(), Box<dyn std::error::Error>> {
    let prev = transact_retry(
        port,
        &build_command("VNRRG,07"),
        5,
        |l| parse_reg07(l).is_some(),
        "could not read current async rate",
    )?;
    let prev_hz = parse_reg07(&prev).unwrap();

    // Optionally switch the ASCII message type (register 6), remembering the
    // previous value to restore it.
    let prev_type = if let Some(t) = ascii_type {
        let r = transact_retry(
            port,
            &build_command("VNRRG,06"),
            5,
            |l| parse_reg06(l).is_some(),
            "could not read current ASCII type",
        )?;
        let prev_t = parse_reg06(&r).unwrap();
        transact_retry(
            port,
            &build_command(&format!("VNWRG,06,{t}")),
            5,
            |l| l.starts_with("$VNWRG,06"),
            "device did not accept the ASCII type (register 6)",
        )?;
        println!("Set ASCII type to {} (ADOR {t}).", ascii_type_name(t));
        Some(prev_t)
    } else {
        None
    };

    // A $VNERR here means the ASCII message doesn't fit at this baud/rate.
    transact_retry(
        port,
        &build_command(&format!("VNWRG,07,{hz}")),
        5,
        |l| parse_reg07(l).is_some(),
        "device did not accept the async rate (a $VNERR means the message won't fit at this baud)",
    )?;
    println!("Set ASCII async rate to {hz} Hz; measuring the $VN message stream for {secs}s...");

    let measured = measure_ascii(port, secs)?;
    let msgs = measured.0;
    report_bench("messages", hz, measured, baud);

    let _ = transact_retry(
        port,
        &build_command(&format!("VNWRG,07,{prev_hz}")),
        3,
        |l| parse_reg07(l).is_some(),
        "restore: ASCII async rate",
    );
    if let Some(pt) = prev_type {
        let _ = transact_retry(
            port,
            &build_command(&format!("VNWRG,06,{pt}")),
            3,
            |l| l.starts_with("$VNWRG,06"),
            "restore: ASCII type",
        );
        println!(
            "Restored: ASCII async back to {prev_hz} Hz, type {}.",
            ascii_type_name(pt)
        );
    } else {
        println!("Restored: ASCII async back to {prev_hz} Hz.");
    }

    if msgs == 0 {
        return Err(
            "received 0 async messages — is async output (register 6) on, or is the port wrong?"
                .into(),
        );
    }
    Ok(())
}
