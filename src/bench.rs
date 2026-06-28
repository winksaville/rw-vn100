//! The `bench` command — passively measure whatever the device is already
//! streaming.
//!
//! - `bench` does **no** device writes: it reads the live serial stream for a
//!   fixed window and reports what it sees. No configure, no restore.
//! - The same bytes are scanned two ways at once — ASCII `$VN…` async lines
//!   (checksum-valid) and binary `0xFA` Common-group frames (CRC-valid) — plus
//!   a count of every byte off the wire for total throughput.
//! - Passive means the binary frame length is unknown up front, so each frame
//!   is sniffed from its own header (`try_common_frame`) rather than a config.

use std::io::{ErrorKind, Read};
use std::time::{Duration, Instant};

use crate::proto::*;

// Binary frame: sync 0xFA, groups=0x01 (Common), one 16-bit field mask, the
// streamed fields' payload, then a 2-byte CRC. Length depends on the field set.
const BENCH_SYNC: u8 = 0xFA;
const BENCH_GROUPS: u8 = 0x01;
const BENCH_HEADER: usize = 1 + 1 + 2; // sync + groups + field mask
const BENCH_CRC: usize = 2;

/// What a passive bench run observed over its window.
///
/// - ASCII fields cover the `$VN…` async line stream.
/// - Binary fields cover the `0xFA` Common-group frame stream; `bin_mask` and
///   `bin_first` come from the first valid frame (the field set is constant).
/// - `total_bytes` is every byte read — both streams plus any line noise — for
///   the wire-utilization figure.
struct Stats {
    msgs: u64,
    ascii_bytes: u64,
    ascii_sample: Option<String>,
    frames: u64,
    bin_bytes: u64,
    bin_mask: Option<u16>,
    bin_first: Option<Vec<u8>>,
    total_bytes: u64,
    elapsed: f64,
}

/// Outcome of sniffing a possible binary frame at the start of a slice.
///
/// - `Frame(len)` — a CRC-valid Common frame of `len` bytes starts here.
/// - `NeedMore` — a plausible frame prefix, but too short to decide yet.
/// - `NotHere` — not a frame start (false sync, other group, or an unknown
///   field bit we cannot size).
enum Sniff {
    Frame(usize),
    NeedMore,
    NotHere,
}

/// Read a little-endian f32 from `buf` at `off`.
fn rd_f32(buf: &[u8], off: usize) -> f32 {
    f32::from_le_bytes(buf[off..off + 4].try_into().unwrap()) // OK: caller slices a full frame
}

/// Sniff one Common-group binary frame at the start of `s`.
///
/// Passive: the frame length is read from the frame's own header — sync `0xFA`,
/// groups byte `0x01` (Common), a 16-bit field mask — by summing the known
/// fields' sizes for the payload, then the CRC confirms the guess.
fn try_common_frame(s: &[u8]) -> Sniff {
    if s.is_empty() || s[0] != BENCH_SYNC {
        return Sniff::NotHere;
    }
    if s.len() < 2 {
        return Sniff::NeedMore;
    }
    if s[1] != BENCH_GROUPS {
        return Sniff::NotHere; // only the Common group is supported
    }
    if s.len() < BENCH_HEADER {
        return Sniff::NeedMore;
    }
    let mask = u16::from_le_bytes([s[2], s[3]]);
    if mask == 0 {
        return Sniff::NotHere; // a real frame always carries at least one field
    }
    // Sum the payload from the known Common fields. A set bit we don't know
    // can't be sized, so we can't find the frame end — treat it as a false sync.
    let mut payload = 0usize;
    let mut known = 0u16;
    for f in FIELDS {
        if mask & (1u16 << f.bit) != 0 {
            payload += f.size;
            known |= 1u16 << f.bit;
        }
    }
    if known != mask {
        return Sniff::NotHere;
    }
    let frame_len = BENCH_HEADER + payload + BENCH_CRC;
    if s.len() < frame_len {
        return Sniff::NeedMore;
    }
    // CRC runs over everything after the sync byte (groups..payload..crc); a
    // correct frame makes it zero. 0xFA can appear in payload, so a CRC miss
    // just means this sync was false.
    if vn_crc16(&s[1..frame_len]) == 0 {
        Sniff::Frame(frame_len)
    } else {
        Sniff::NotHere
    }
}

/// Human-readable decode of the selected fields in one binary frame.
fn decode_binary_sample(frame: &[u8], fields: &[&Field]) -> String {
    let mut off = BENCH_HEADER;
    let mut parts = Vec::new();
    for f in fields {
        let s = match f.name {
            "time" => format!(
                "t={} ns",
                u64::from_le_bytes(frame[off..off + 8].try_into().unwrap()) // OK: field sized 8
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
                "uncomp_accel=[{:.3}, {:.3}, {:.3}] m/s^2 uncomp_gyro=[{:.4}, {:.4}, {:.4}] rad/s",
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

/// True if `line` is an async data message (a `$VN…` line that isn't a command
/// echo or error) with a valid checksum.
fn is_async_line(line: &str) -> bool {
    line.starts_with("$VN")
        && !line.starts_with("$VNRRG")
        && !line.starts_with("$VNWRG")
        && !line.starts_with("$VNERR")
        && verify_checksum(line).is_ok()
}

/// Passively read the live stream for `secs`, scanning the same bytes for ASCII
/// async lines and binary Common frames at once.
///
/// - ASCII: split on `\n`, count checksum-valid `$VN…` messages, keep the first
///   as a sample.
/// - Binary: accumulate bytes and pull out CRC-valid `0xFA` frames via
///   `try_common_frame`, keeping the first frame for a decoded sample.
/// - `total_bytes` counts every byte read, for the wire-utilization figure.
fn measure<S: Read>(port: &mut S, secs: u64) -> std::io::Result<Stats> {
    let start = Instant::now();
    let deadline = start + Duration::from_secs(secs);
    let mut buf = [0u8; 1024];

    let mut total_bytes = 0u64;

    // ASCII line-scanner state.
    let mut line: Vec<u8> = Vec::new();
    let mut msgs = 0u64;
    let mut ascii_bytes = 0u64;
    let mut ascii_sample: Option<String> = None;

    // Binary frame-scanner state.
    let mut acc: Vec<u8> = Vec::new();
    let mut frames = 0u64;
    let mut bin_bytes = 0u64;
    let mut bin_mask: Option<u16> = None;
    let mut bin_first: Option<Vec<u8>> = None;

    while Instant::now() < deadline {
        let n = match port.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(ref e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {
                continue;
            }
            Err(e) => return Err(e),
        };
        let chunk = &buf[..n];
        total_bytes += n as u64;

        // ASCII view: walk bytes, completing a line on each `\n`.
        for &b in chunk {
            match b {
                b'\n' => {
                    let raw = String::from_utf8_lossy(&line);
                    let cand = match raw.rfind('$') {
                        Some(p) => raw[p..].trim(),
                        None => raw.trim(),
                    };
                    if is_async_line(cand) {
                        msgs += 1;
                        ascii_bytes += cand.len() as u64 + 2; // + CRLF
                        if ascii_sample.is_none() {
                            ascii_sample = Some(cand.to_string());
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

        // Binary view: accumulate, then pull out CRC-valid Common frames.
        acc.extend_from_slice(chunk);
        let mut i = 0;
        while i < acc.len() {
            if acc[i] != BENCH_SYNC {
                i += 1;
                continue;
            }
            match try_common_frame(&acc[i..]) {
                Sniff::Frame(flen) => {
                    frames += 1;
                    bin_bytes += flen as u64;
                    if bin_mask.is_none() {
                        bin_mask = Some(u16::from_le_bytes([acc[i + 2], acc[i + 3]]));
                        bin_first = Some(acc[i..i + flen].to_vec());
                    }
                    i += flen;
                }
                Sniff::NotHere => i += 1,
                Sniff::NeedMore => break, // partial frame at the tail; wait for more
            }
        }
        acc.drain(0..i);
        // Backstop: a stream with no valid frames never drains its tail prefix,
        // so cap the buffer (keep enough to span any single frame).
        if acc.len() > 8192 {
            let cut = acc.len() - 512;
            acc.drain(0..cut);
        }
    }

    Ok(Stats {
        msgs,
        ascii_bytes,
        ascii_sample,
        frames,
        bin_bytes,
        bin_mask,
        bin_first,
        total_bytes,
        elapsed: start.elapsed().as_secs_f64(),
    })
}

/// Print the passive bench result: ASCII rate, binary rate, wire throughput.
fn report(baud: u32, st: &Stats) {
    let secs = st.elapsed;
    let rate = |count: u64| if secs > 0.0 { count as f64 / secs } else { 0.0 };
    println!();

    if st.msgs > 0 {
        println!(
            "ASCII:  {} messages in {secs:.2}s = {:.1} Hz ({} B/s).",
            st.msgs,
            rate(st.msgs),
            (st.ascii_bytes as f64 / secs).round() as u64
        );
        if let Some(s) = &st.ascii_sample {
            println!("  Sample: {s}");
        }
    } else {
        println!("ASCII:  none seen.");
    }

    if st.frames > 0 {
        let mask = st.bin_mask.unwrap_or(0); // OK: frames>0 implies a mask was captured
        let fields = fields_from_mask(mask);
        let names: Vec<&str> = fields.iter().map(|f| f.name).collect();
        let flen = st.bin_first.as_ref().map(|f| f.len()).unwrap_or(0); // OK: frames>0 implies a frame
        println!(
            "Binary: {} frames in {secs:.2}s = {:.1} Hz ({} B/s).",
            st.frames,
            rate(st.frames),
            (st.bin_bytes as f64 / secs).round() as u64
        );
        println!("  Fields: Common{names:?} ({flen} B/frame).");
        if let Some(fr) = &st.bin_first {
            println!("  Sample: {}", decode_binary_sample(fr, &fields));
        }
    } else {
        println!("Binary: none seen.");
    }

    let bytes_per_s = if secs > 0.0 {
        st.total_bytes as f64 / secs
    } else {
        0.0
    };
    let bits = bytes_per_s * 10.0;
    let pct = 100.0 * bits / baud as f64;
    println!(
        "Wire throughput ~{:.0} B/s = ~{:.0} kbit/s = {:.0}% of the {:.1} kbit/s {baud}-baud link.",
        bytes_per_s,
        bits / 1000.0,
        pct,
        baud as f64 / 1000.0
    );
}

/// Passively measure the device's live output for `secs` seconds.
///
/// - Does no device writes — reports whatever is already streaming.
/// - Counts ASCII `$VN` async lines and binary `0xFA` Common-group frames
///   (CRC-checked) over the same bytes, plus total wire throughput against the
///   `baud` link.
pub fn bench<S: Read>(
    port: &mut S,
    baud: u32,
    secs: u64,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Measuring the live stream for {secs}s (passive — no device writes)...");
    let st = measure(port, secs)?;
    report(baud, &st);
    if st.msgs == 0 && st.frames == 0 {
        return Err(
            "saw no ASCII messages or binary frames — is the device streaming, \
             and does --baud match its current rate?"
                .into(),
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `Read` that hands out a fixed buffer in `chunk`-sized pieces (to
    /// simulate a real port splitting frames/lines across reads), then signals
    /// EOF (`Ok(0)`) so `measure` idles until its deadline.
    struct MockReader {
        data: Vec<u8>,
        pos: usize,
        chunk: usize,
    }

    impl Read for MockReader {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let n = (self.data.len() - self.pos).min(buf.len()).min(self.chunk);
            buf[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
            self.pos += n;
            Ok(n)
        }
    }

    /// Build a CRC-valid Common frame for `mask` with zeroed payload.
    fn test_frame(mask: u16) -> Vec<u8> {
        let payload: usize = fields_from_mask(mask).iter().map(|f| f.size).sum();
        let mut data = vec![BENCH_GROUPS, (mask & 0xff) as u8, (mask >> 8) as u8];
        data.extend(std::iter::repeat_n(0u8, payload));
        let crc = vn_crc16(&data);
        let mut frame = vec![BENCH_SYNC];
        frame.extend_from_slice(&data);
        frame.extend_from_slice(&crc.to_be_bytes()); // appended big-endian → CRC over frame == 0
        frame
    }

    /// Build the device-shaped interleaved stream: `reps` binary frames (full
    /// 7-field, like the real 110-byte frame) each followed by an ASCII line.
    fn interleaved_stream(reps: usize) -> Vec<u8> {
        let frame = test_frame(0x0739); // all seven Common fields
        let payload = "VNYMR,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,1.0,2.0,3.0";
        let ascii = format!("${payload}*{:02X}\r\n", checksum(payload));
        let mut data = Vec::new();
        for _ in 0..reps {
            data.extend_from_slice(&frame);
            data.extend_from_slice(ascii.as_bytes());
        }
        data
    }

    #[test]
    fn measure_parses_real_both_streams_capture() {
        // ~1 s of real wire data: binary @ 200 Hz + ASCII VNYMR @ 40 Hz.
        let data = include_bytes!("../test-data/both-streams.bin").to_vec();
        for chunk in [usize::MAX, 1, 64, 512] {
            let mut reader = MockReader {
                data: data.clone(),
                pos: 0,
                chunk,
            };
            let st = measure(&mut reader, 1).unwrap();
            assert!(
                st.frames > 150 && st.msgs > 20,
                "chunk={chunk}: frames={} msgs={}",
                st.frames,
                st.msgs
            );
        }
    }

    #[test]
    fn measure_counts_interleaved_streams_one_read() {
        let data = interleaved_stream(5);
        let mut reader = MockReader {
            data,
            pos: 0,
            chunk: usize::MAX,
        };
        let st = measure(&mut reader, 1).unwrap();
        assert_eq!(st.frames, 5, "binary frames");
        assert_eq!(st.msgs, 5, "ascii messages");
    }

    #[test]
    fn measure_counts_interleaved_streams_fragmented() {
        // Small reads split frames and lines across reads, like a real port.
        for chunk in [1usize, 7, 13, 64] {
            let data = interleaved_stream(20);
            let mut reader = MockReader {
                data,
                pos: 0,
                chunk,
            };
            let st = measure(&mut reader, 1).unwrap();
            assert_eq!(st.frames, 20, "binary frames at chunk={chunk}");
            assert_eq!(st.msgs, 20, "ascii messages at chunk={chunk}");
        }
    }
}
