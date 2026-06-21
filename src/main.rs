//! Read or set the Async Data Output Frequency (register 7) on a VectorNav VN-100,
//! and change the device's serial baud rate (register 5).
//!
//! The VN-100 speaks an ASCII protocol over a serial port. Each command is
//!
//!     $<payload>*XX\r\n
//!
//! where `XX` is the 8-bit XOR checksum of every character of `<payload>`
//! (i.e. everything between `$` and `*`).
//!
//! Read Register 7:   $VNRRG,07*XX        -> reply $VNRRG,07,<freq>*YY
//! Write Register 7:  $VNWRG,07,<freq>*XX -> reply $VNWRG,07,<freq>*YY
//! Write Register 5:  $VNWRG,05,<baud>*XX -> reply $VNWRG,05,<baud>*YY  (serial baud)
//! Write Settings:    $VNWNV*XX           -> reply $VNWNV*YY            (save to flash)
//! Error response:    $VNERR,<code>*XX
//!
//! `<freq>` is the async output rate in Hz. The VN-100 has no command to query
//! the allowable rates: the set is fixed in firmware (see `VALID_RATES`), and
//! writing an out-of-range value returns a `$VNERR` response.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

/// Frequencies (Hz) the VN-100 accepts for the async data output rate.
const VALID_RATES: &[u32] = &[1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200];

/// Serial baud rates the VN-100 supports (register 5).
const VALID_BAUDS: &[u32] = &[
    9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600,
];

/// Compute the VN-100 checksum: XOR of all bytes in `payload`.
fn checksum(payload: &str) -> u8 {
    payload.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// Build a full command line (including `$`, `*`, checksum and CRLF) from the
/// payload that sits between `$` and `*`, e.g. `"VNRRG,07"`.
fn build_command(payload: &str) -> String {
    format!("${}*{:02X}\r\n", payload, checksum(payload))
}

/// Verify the trailing `*XX` checksum of a received `$...*XX` line.
fn verify_checksum(line: &str) -> Result<(), String> {
    let payload = line.strip_prefix('$').ok_or("reply missing leading '$'")?;
    let (payload, sum) = payload
        .rsplit_once('*')
        .ok_or("reply missing '*' checksum delimiter")?;
    let given = u8::from_str_radix(sum.trim(), 16)
        .map_err(|_| format!("malformed checksum field {sum:?}"))?;
    let actual = checksum(payload);
    if given == actual {
        Ok(())
    } else {
        Err(format!(
            "checksum mismatch: reply says {given:02X}, computed {actual:02X}"
        ))
    }
}

/// Human-readable description of a VN-100 system error code.
fn error_description(code: u8) -> &'static str {
    match code {
        1 => "hard fault",
        2 => "serial buffer overflow",
        3 => "invalid checksum",
        4 => "invalid command",
        5 => "not enough parameters",
        6 => "too many parameters",
        7 => "invalid parameter",
        8 => "invalid register",
        9 => "unauthorized access",
        10 => "watchdog reset",
        11 => "output buffer overflow",
        12 => "insufficient baud rate",
        255 => "error buffer overflow",
        _ => "unknown error",
    }
}

/// Turn a `$VNERR,<code>*XX` line into a readable message. The code is hex.
fn vnerr_message(line: &str) -> String {
    let code = line
        .strip_prefix("$VNERR,")
        .and_then(|b| b.split('*').next())
        .map(str::trim)
        .and_then(|c| u8::from_str_radix(c, 16).ok());
    match code {
        Some(n) => {
            let mut msg = format!("device error 0x{n:02X} ({n}): {}", error_description(n));
            if n == 12 {
                // By far the most likely error when setting a high output rate.
                msg.push_str(
                    " — the configured async message won't fit at this output rate over the \
                     current serial baud; raise the baud (e.g. `baud 921600`) or shorten the \
                     async message (register 6)",
                );
            }
            msg
        }
        None => format!("device error: {line}"),
    }
}

/// Parse the rate out of a `$VN(R|W)RG,07,<freq>*XX` response line.
/// Returns `None` for any other (e.g. async) line.
fn parse_reg07(line: &str) -> Option<u32> {
    let body = line
        .strip_prefix("$VNRRG,07,")
        .or_else(|| line.strip_prefix("$VNWRG,07,"))?;
    let freq = body.split('*').next()?;
    freq.trim().parse().ok()
}

struct Config {
    port: String,
    baud: u32,
}

enum Command {
    Help,
    Get,
    Set {
        hz: u32,
        persist: bool,
    },
    SetBaud {
        baud: u32,
        persist: bool,
    },
    Reset,
    FactoryReset,
    /// Configure a compact binary output and measure the achieved frame rate.
    Bench {
        hz: u32,
        secs: u64,
    },
}

fn help_text() -> String {
    format!(
        "rdwr_vn100 - read/set the VN-100 async output rate (reg 7) and serial baud (reg 5)\n\n\
         Usage:\n  \
           rdwr_vn100 [--port PORT] [--baud BAUD] get\n  \
           rdwr_vn100 [--port PORT] [--baud BAUD] set <HZ> [--persist]\n  \
           rdwr_vn100 [--port PORT] [--baud BAUD] baud <NEW_BAUD> [--persist]\n  \
           rdwr_vn100 [--port PORT] [--baud BAUD] reset\n  \
           rdwr_vn100 [--port PORT] [--baud BAUD] factory-reset\n  \
           rdwr_vn100 [--port PORT] [--baud BAUD] bench [--hz HZ] [--secs S]\n  \
           rdwr_vn100 help | --help | -h\n\n\
         Commands:\n  \
           get             Read the current async output rate.\n  \
           set <HZ>        Write the async output rate.\n  \
           baud <NEW_BAUD> Change the device's serial baud rate (register 5), then\n  \
                           switch this connection to it and verify, all without\n  \
                           closing the port.\n  \
           reset           Reboot the sensor ($VNRST); reloads saved flash settings.\n  \
           factory-reset   Restore ALL registers to factory defaults and reboot\n  \
                           ($VNRFS). Reverts baud to 115200 and async output to\n  \
                           default. Not undoable.\n  \
           bench           Configure a compact binary output (Common: TimeStartup +\n  \
                           Accel) at HZ and measure the achieved frame rate, then\n  \
                           restore prior state. Proves a high rate fits the link.\n\n\
         Bench options:\n  \
           --hz HZ      Target binary rate; must divide 800 (default 200).\n  \
           --secs S     Measurement duration in seconds (default 5).\n\n\
         Options:\n  \
           --port PORT  Serial device (default: /dev/ttyUSB0)\n  \
           --baud BAUD  Baud rate to talk to the device at NOW (default: 115200);\n  \
                        must match the device's CURRENT rate.\n  \
           --persist    Save settings to flash so they survive a power cycle\n  \
                        (works with `set` and `baud`).\n\n\
         Valid HZ:   {VALID_RATES:?}\n  \
           Fixed in firmware; the VN-100 has no command to query them, and rejects\n  \
           out-of-range values with a $VNERR response.\n\
         Valid BAUD: {VALID_BAUDS:?}\n\n\
         Note: a baud change is volatile — the device keeps it across host reconnects,\n  \
           but a power cycle or reset reverts to the flash baud. Persist to keep it:\n    \
             rdwr_vn100 baud 921600 --persist        # change + verify + save to flash\n    \
             rdwr_vn100 --baud 921600 get            # device now boots at 921600\n\n\
         Examples:\n  \
           rdwr_vn100 get\n  \
           rdwr_vn100 set 40 --persist\n  \
           rdwr_vn100 --port /dev/ttyACM0 --baud 921600 get\n"
    )
}

/// Parse CLI args into a connection config and a command.
fn parse_args<I: Iterator<Item = String>>(args: I) -> Result<(Config, Command), String> {
    let args: Vec<String> = args.collect();
    if args
        .iter()
        .any(|a| matches!(a.as_str(), "help" | "--help" | "-h"))
    {
        return Ok((
            Config {
                port: String::new(),
                baud: 0,
            },
            Command::Help,
        ));
    }

    let mut port = "/dev/ttyUSB0".to_string();
    let mut baud = 115_200u32;
    let mut persist = false;
    let mut hz: Option<u32> = None;
    let mut secs: Option<u64> = None;
    let mut positional: Vec<String> = Vec::new();

    let mut args = args.into_iter();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--port" => port = args.next().ok_or("--port requires a value")?,
            "--baud" => {
                baud = args
                    .next()
                    .ok_or("--baud requires a value")?
                    .parse()
                    .map_err(|_| "--baud must be a number")?
            }
            "--persist" => persist = true,
            "--hz" => {
                hz = Some(
                    args.next()
                        .ok_or("--hz requires a value")?
                        .parse()
                        .map_err(|_| "--hz must be a number")?,
                )
            }
            "--secs" => {
                secs = Some(
                    args.next()
                        .ok_or("--secs requires a value")?
                        .parse()
                        .map_err(|_| "--secs must be a number")?,
                )
            }
            _ => positional.push(arg),
        }
    }

    let command = match positional.first().map(String::as_str) {
        Some("get") => {
            if persist {
                return Err("--persist only applies to `set`".into());
            }
            Command::Get
        }
        Some("set") => {
            let hz: u32 = positional
                .get(1)
                .ok_or("set requires a frequency, e.g. `set 40`")?
                .parse()
                .map_err(|_| "frequency must be a number")?;
            if !VALID_RATES.contains(&hz) {
                return Err(format!(
                    "{hz} Hz is not valid; choose one of {VALID_RATES:?}"
                ));
            }
            Command::Set { hz, persist }
        }
        Some("baud") => {
            let new_baud: u32 = positional
                .get(1)
                .ok_or("baud requires a value, e.g. `baud 921600`")?
                .parse()
                .map_err(|_| "baud must be a number")?;
            if !VALID_BAUDS.contains(&new_baud) {
                return Err(format!(
                    "{new_baud} is not a valid VN-100 baud; choose one of {VALID_BAUDS:?}"
                ));
            }
            Command::SetBaud {
                baud: new_baud,
                persist,
            }
        }
        Some("reset") => Command::Reset,
        Some("factory-reset") => Command::FactoryReset,
        Some("bench") => {
            let hz = hz.unwrap_or(200);
            if hz == 0 || 800 % hz != 0 {
                return Err(format!(
                    "--hz {hz} invalid; the binary rate is 800/divisor, so HZ must divide 800 \
                     (e.g. 100, 200, 400)"
                ));
            }
            Command::Bench {
                hz,
                secs: secs.unwrap_or(5),
            }
        }
        Some(other) => return Err(format!("unknown command `{other}`")),
        None => {
            return Err(
                "missing command (`get`, `set`, `baud`, `reset`, `factory-reset`, or `help`)"
                    .into(),
            )
        }
    };

    Ok((Config { port, baud }, command))
}

/// Read from the device until `matches` accepts a line or the deadline passes.
///
/// Reads raw bytes (not `read_line`) so a stream of garbage with no newline —
/// e.g. when the host baud doesn't match the device — can't block us forever:
/// we honor an overall `deadline` and cap line length to drop runaway junk.
fn read_reply<R, F>(
    reader: &mut R,
    deadline: Instant,
    mut matches: F,
) -> std::io::Result<Option<String>>
where
    R: Read,
    F: FnMut(&str) -> bool,
{
    let mut buf = [0u8; 256];
    let mut line: Vec<u8> = Vec::new();
    loop {
        if Instant::now() >= deadline {
            return Ok(None);
        }
        let n = match reader.read(&mut buf) {
            // On a serial port there is no real EOF: a read that yields nothing
            // (Ok(0), or a TimedOut/WouldBlock error) just means "no data within
            // this read window". Keep waiting — the overall `deadline` is the only
            // terminator, so a reply that lags (e.g. while the device reconfigures
            // its UART for a baud change) isn't dropped.
            Ok(0) => continue,
            Ok(n) => n,
            Err(ref e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                ) =>
            {
                continue
            }
            Err(e) => return Err(e),
        };
        for &b in &buf[..n] {
            match b {
                b'\n' => {
                    let raw = String::from_utf8_lossy(&line);
                    // An ASCII reply ($VN...*XX) can arrive with leading bytes on
                    // the same line — e.g. binary frames still streaming when the
                    // echo lands. The reply starts at the last '$', so slice there
                    // before matching/validating.
                    let candidate: String = match raw.rfind('$') {
                        Some(p) => raw[p..].trim().to_string(),
                        None => raw.trim().to_string(),
                    };
                    line.clear();
                    if matches(&candidate) {
                        return Ok(Some(candidate));
                    }
                }
                b'\r' => {}
                _ => {
                    line.push(b);
                    if line.len() > 1024 {
                        line.clear(); // drop a runaway (likely garbage) line
                    }
                }
            }
        }
    }
}

/// Send `cmd`, wait up to `wait` for a reply matching `accept`, and validate it.
/// A `$VNERR` reply is surfaced as an error; a checksum mismatch is rejected.
fn transact<S: Read + Write>(
    port: &mut S,
    cmd: &str,
    wait: Duration,
    accept: impl Fn(&str) -> bool,
    missing: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    print!("TX: {cmd}");
    port.write_all(cmd.as_bytes())?;
    port.flush()?;

    let deadline = Instant::now() + wait;
    let reply = read_reply(port, deadline, |l| accept(l) || l.starts_with("$VNERR"))?
        .ok_or_else(|| missing.to_string())?;
    verify_checksum(&reply)?;
    if reply.starts_with("$VNERR") {
        return Err(vnerr_message(&reply).into());
    }
    Ok(reply)
}

/// Like `transact`, but resends the command up to `attempts` times. A fresh
/// open (especially at high baud) can drop the first query or its reply while
/// the USB-serial chip locks its divisor and open-time line noise drains, so a
/// single shot is unreliable. A device `$VNERR` is returned immediately —
/// retrying a rejected command won't change the answer.
fn transact_retry<S: Read + Write>(
    port: &mut S,
    cmd: &str,
    attempts: u32,
    accept: impl Fn(&str) -> bool,
    missing: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    let mut last: Option<Box<dyn std::error::Error>> = None;
    for attempt in 1..=attempts {
        match transact(
            port,
            cmd,
            Duration::from_millis(800),
            |l| accept(l),
            "no reply yet",
        ) {
            Ok(reply) => return Ok(reply),
            // A device-side error is a definitive answer; don't keep retrying it.
            Err(e) if e.to_string().starts_with("device error") => return Err(e),
            Err(e) => {
                last = Some(e);
                if attempt < attempts {
                    println!("  attempt {attempt}/{attempts}: no response yet, retrying...");
                    std::thread::sleep(Duration::from_millis(100));
                }
            }
        }
    }
    Err(format!(
        "{missing} (after {attempts} attempts; last: {})",
        last.map(|e| e.to_string()).unwrap_or_default()
    )
    .into())
}

/// Send a command that causes the device to reboot ($VNRST / $VNRFS). The echo
/// may or may not arrive before the reset, so a missing reply is NOT an error;
/// only a `$VNERR` (e.g. unauthorized) is surfaced.
fn send_reboot_command<S: Read + Write>(
    port: &mut S,
    cmd: &str,
    accept: impl Fn(&str) -> bool,
) -> Result<(), Box<dyn std::error::Error>> {
    print!("TX: {cmd}");
    port.write_all(cmd.as_bytes())?;
    port.flush()?;

    let deadline = Instant::now() + Duration::from_millis(1500);
    match read_reply(port, deadline, |l| accept(l) || l.starts_with("$VNERR"))? {
        Some(reply) if reply.starts_with("$VNERR") => Err(vnerr_message(&reply).into()),
        Some(reply) => {
            println!("RX: {reply}");
            Ok(())
        }
        None => {
            println!("(no echo — device likely rebooted before replying, which is normal)");
            Ok(())
        }
    }
}

/// VectorNav 16-bit CRC (CRC-CCITT/XMODEM, the algorithm from their app note).
/// A valid binary packet, run from the groups byte through the trailing CRC,
/// produces 0.
fn vn_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        crc = crc.rotate_left(8);
        crc ^= b as u16;
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
    }
    crc
}

// Our bench binary frame: sync 0xFA, groups=0x01 (Common), fields=0x0101
// (TimeStartup[8] + Accel[12]), then 2-byte CRC. Fixed layout => fixed length.
const BENCH_SYNC: u8 = 0xFA;
const BENCH_GROUPS: u8 = 0x01;
const BENCH_FRAME_LEN: usize = 1 + 1 + 2 + 8 + 12 + 2; // = 26

/// One decoded sample: (timestamp_ns, accel_x, accel_y, accel_z).
type AccelSample = (u64, f32, f32, f32);

/// Outcome of a binary-rate measurement.
struct BenchResult {
    frames: u64,
    elapsed: f64,
    sample: Option<AccelSample>,
}

/// Read the binary stream for `secs` seconds, counting CRC-valid frames.
fn measure_binary<S: Read>(port: &mut S, secs: u64) -> std::io::Result<BenchResult> {
    let start = Instant::now();
    let deadline = start + Duration::from_secs(secs);
    let mut buf = [0u8; 1024];
    let mut acc: Vec<u8> = Vec::new();
    let mut frames = 0u64;
    let mut sample = None;

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
                continue
            }
            Err(e) => return Err(e),
        }

        let mut i = 0;
        while i + BENCH_FRAME_LEN <= acc.len() {
            if acc[i] != BENCH_SYNC || acc[i + 1] != BENCH_GROUPS {
                i += 1;
                continue;
            }
            let frame = &acc[i..(i + BENCH_FRAME_LEN)];
            // CRC over everything after the sync byte (groups..payload..crc) == 0.
            if vn_crc16(&frame[1..]) == 0 {
                frames += 1;
                if sample.is_none() {
                    let t = u64::from_le_bytes(frame[4..12].try_into().unwrap());
                    let ax = f32::from_le_bytes(frame[12..16].try_into().unwrap());
                    let ay = f32::from_le_bytes(frame[16..20].try_into().unwrap());
                    let az = f32::from_le_bytes(frame[20..24].try_into().unwrap());
                    sample = Some((t, ax, ay, az));
                }
                i += BENCH_FRAME_LEN;
            } else {
                i += 1; // false sync (0xFA can appear in payload); resync
            }
        }
        acc.drain(0..i);
        if acc.len() > 8192 {
            // Bound memory if we're somehow not finding frames.
            let keep = acc.len() - BENCH_FRAME_LEN;
            acc.drain(0..keep);
        }
    }

    Ok(BenchResult {
        frames,
        elapsed: start.elapsed().as_secs_f64(),
        sample,
    })
}

/// Configure a compact binary output (Common: TimeStartup + Accel) at `hz`,
/// measure the achieved frame rate for `secs`, then restore the prior state.
fn run_bench<S: Read + Write>(
    port: &mut S,
    baud: u32,
    hz: u32,
    secs: u64,
) -> Result<(), Box<dyn std::error::Error>> {
    let divisor = 800 / hz; // device IMU base rate is 800 Hz

    // Remember the current ASCII async rate so we can put it back.
    let prev = transact_retry(
        port,
        &build_command("VNRRG,07"),
        5,
        |l| parse_reg07(l).is_some(),
        "could not read current async rate",
    )?;
    let prev_hz = parse_reg07(&prev).unwrap();
    println!("Current ASCII async rate: {prev_hz} Hz (will restore afterward).");

    // Silence the ASCII async output so we measure ONLY the binary stream.
    transact_retry(
        port,
        &build_command("VNWRG,07,0"),
        5,
        |l| parse_reg07(l).is_some(),
        "could not disable ASCII async output",
    )?;

    // Binary Output 1 (reg 75): serial1, divisor, Common group, TimeStartup+Accel.
    let cfg = format!("VNWRG,75,1,{divisor},01,0101");
    println!("TX config: ${cfg}*..");
    transact_retry(
        port,
        &build_command(&cfg),
        5,
        |l| l.starts_with("$VNWRG,75"),
        "device did not accept the binary output config (a $VNERR here would mean it won't fit)",
    )?;
    println!(
        "Configured binary output: Common[TimeStartup, Accel] @ {} Hz (divisor {divisor}, {} bytes/frame).",
        800 / divisor,
        BENCH_FRAME_LEN
    );

    // No explicit buffer flush here: the frame parser CRC-validates and resyncs,
    // so the config echo and any partial leading bytes are simply skipped.
    println!("Measuring for {secs}s...");
    let BenchResult {
        frames,
        elapsed,
        sample,
    } = measure_binary(port, secs)?;
    let rate = if elapsed > 0.0 {
        frames as f64 / elapsed
    } else {
        0.0
    };

    println!(
        "\nResult: {frames} valid frames in {elapsed:.2}s = {rate:.1} Hz (target {} Hz).",
        800 / divisor
    );
    if let Some((t, ax, ay, az)) = sample {
        println!("Sample frame: t={t} ns, accel = [{ax:.3}, {ay:.3}, {az:.3}] m/s^2");
    }
    // ~10 bits/byte on the wire (8N1); baud == bits/s for UART.
    let bits_per_sec = rate * BENCH_FRAME_LEN as f64 * 10.0;
    let pct = 100.0 * bits_per_sec / baud as f64;
    println!(
        "Wire throughput ~{:.0} kbit/s = {:.0}% of the {:.1} kbit/s {baud}-baud link.",
        bits_per_sec / 1000.0,
        pct,
        baud as f64 / 1000.0
    );

    // Restore: turn the binary output off, put the ASCII rate back.
    let _ = transact_retry(
        port,
        &build_command(&format!("VNWRG,75,0,{divisor},01,0101")),
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (config, command) = match parse_args(std::env::args().skip(1)) {
        Ok(parsed) => parsed,
        Err(e) => {
            eprintln!("error: {e}\n");
            eprint!("{}", help_text());
            std::process::exit(2);
        }
    };

    if let Command::Help = command {
        print!("{}", help_text());
        return Ok(());
    }

    println!("Opening {} at {} baud...", config.port, config.baud);
    let mut port = serialport::new(&config.port, config.baud)
        // Short per-read timeout so read_reply re-checks its overall deadline often.
        .timeout(Duration::from_millis(250))
        .open()?;

    // Let the USB-serial chip lock its baud divisor, then drop any open-time
    // line noise / partial async frame before the first command. Without this,
    // a fresh open at high baud often loses its first query.
    std::thread::sleep(Duration::from_millis(150));
    let _ = port.clear(serialport::ClearBuffer::Input);

    // Shared "we heard nothing usable" hint — usually a baud mismatch.
    let no_reply = format!(
        "no usable reply from device — is it actually at {} baud? \
         (VN-100 factory default is 115200; use --baud to match, or the `baud` command to change it)",
        config.baud
    );

    match command {
        Command::Help => unreachable!("handled above"),

        Command::Get => {
            let reply = transact_retry(
                &mut port,
                &build_command("VNRRG,07"),
                5,
                |l| parse_reg07(l).is_some(),
                &no_reply,
            )?;
            println!("RX: {reply}");
            println!("Async output rate: {} Hz", parse_reg07(&reply).unwrap());
        }

        Command::Set { hz, persist } => {
            let reply = transact_retry(
                &mut port,
                &build_command(&format!("VNWRG,07,{hz}")),
                5,
                |l| parse_reg07(l).is_some(),
                &no_reply,
            )?;
            println!("RX: {reply}");
            println!("Async output rate: {} Hz", parse_reg07(&reply).unwrap());

            if persist {
                let confirm = transact_retry(
                    &mut port,
                    &build_command("VNWNV"),
                    5,
                    |l| l.starts_with("$VNWNV"),
                    &no_reply,
                )?;
                println!("RX: {confirm}");
                println!("Settings written to non-volatile memory.");
            }
        }

        Command::SetBaud {
            baud: new_baud,
            persist,
        } => {
            // The device replies at the CURRENT baud, then switches to the new one.
            let reply = transact_retry(
                &mut port,
                &build_command(&format!("VNWRG,05,{new_baud}")),
                5,
                |l| l.starts_with("$VNWRG,05,"),
                &no_reply,
            )?;
            println!("RX: {reply}");
            println!("Device acknowledged baud change to {new_baud}.");

            // Give the device a moment to reconfigure its UART before we talk at
            // the new rate (the vendor SDK waits ~50 ms here), then switch THIS
            // connection in place. We switch in-session rather than close/reopen
            // not because the device would forget the baud — it holds the RAM
            // value across host reconnects — but because each reconnect risks a
            // line transient that, at very high baud (e.g. 921600 on an FT232R),
            // can wedge the link until a power cycle.
            std::thread::sleep(Duration::from_millis(60));
            port.set_baud_rate(new_baud)?;
            // Drop any bytes that were in flight across the switch.
            let _ = port.clear(serialport::ClearBuffer::Input);

            println!("Verifying at {new_baud} baud...");
            let verify = transact_retry(
                &mut port,
                &build_command("VNRRG,07"),
                5,
                |l| parse_reg07(l).is_some(),
                "device did not respond at the new baud (a power cycle reverts to the flash baud)",
            )?;
            println!("RX: {verify}");
            println!(
                "Verified — device is at {new_baud} baud (async rate {} Hz).",
                parse_reg07(&verify).unwrap()
            );

            if persist {
                let confirm = transact_retry(
                    &mut port,
                    &build_command("VNWNV"),
                    5,
                    |l| l.starts_with("$VNWNV"),
                    "no $VNWNV confirmation at the new baud",
                )?;
                println!("RX: {confirm}");
                println!("Baud saved to flash; the device will boot at {new_baud} from now on.");
            } else {
                println!(
                    "(Volatile — the device holds this across host reconnects, but a \
                     power cycle or `reset`/`factory-reset` reverts it to the flash baud. \
                     Re-run with `baud {new_baud} --persist` to make it permanent.)"
                );
            }
        }

        Command::Reset => {
            send_reboot_command(&mut port, &build_command("VNRST"), |l| {
                l.starts_with("$VNRST")
            })?;
            println!("Reset requested — sensor is rebooting and reloading its saved settings.");
        }

        Command::Bench { hz, secs } => {
            run_bench(&mut port, config.baud, hz, secs)?;
        }

        Command::FactoryReset => {
            println!("Restoring factory defaults — this overwrites flash and cannot be undone.");
            send_reboot_command(&mut port, &build_command("VNRFS"), |l| {
                l.starts_with("$VNRFS")
            })?;
            println!("Factory restore requested — sensor is rebooting.");
            println!(
                "It is now at 115200 baud with the default async output. \
                 Reconnect with the default --baud (115200)."
            );
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_matches_known_value() {
        assert_eq!(format!("{:02X}", checksum("VNRRG,07")), "74");
    }

    #[test]
    fn builds_read_command() {
        assert_eq!(build_command("VNRRG,07"), "$VNRRG,07*74\r\n");
    }

    #[test]
    fn builds_write_command() {
        // XOR of "VNWRG,07,40"
        assert_eq!(build_command("VNWRG,07,40"), "$VNWRG,07,40*59\r\n");
    }

    #[test]
    fn verifies_good_checksum() {
        let line = format!("$VNRRG,07,40*{:02X}", checksum("VNRRG,07,40"));
        assert!(verify_checksum(&line).is_ok());
    }

    #[test]
    fn rejects_bad_checksum() {
        assert!(verify_checksum("$VNRRG,07,40*00").is_err());
        assert!(verify_checksum("no dollar*FF").is_err());
        assert!(verify_checksum("$VNRRG,07,40").is_err()); // no '*'
    }

    #[test]
    fn decodes_insufficient_baud_error() {
        let msg = vnerr_message("$VNERR,0C*02");
        assert!(msg.contains("insufficient baud rate"), "{msg}");
        assert!(msg.contains("(12)"), "{msg}");
    }

    #[test]
    fn decodes_other_error_codes() {
        assert_eq!(error_description(7), "invalid parameter");
        assert_eq!(error_description(8), "invalid register");
        assert!(vnerr_message("$VNERR,07*XX").contains("invalid parameter"));
    }

    #[test]
    fn parses_read_and_write_responses() {
        assert_eq!(parse_reg07("$VNRRG,07,40*4C"), Some(40));
        assert_eq!(parse_reg07("$VNWRG,07,100*2B"), Some(100));
        assert_eq!(parse_reg07("$VNYMR,+010.0*7F"), None);
    }

    #[test]
    fn rejects_invalid_set_rate() {
        let args = ["set", "33"].into_iter().map(String::from);
        assert!(parse_args(args).is_err());
    }

    #[test]
    fn parses_flags_and_set_command() {
        let args = ["--port", "/dev/ttyACM0", "--baud", "921600", "set", "40"]
            .into_iter()
            .map(String::from);
        let (config, command) = parse_args(args).unwrap();
        assert_eq!(config.port, "/dev/ttyACM0");
        assert_eq!(config.baud, 921_600);
        assert!(matches!(
            command,
            Command::Set {
                hz: 40,
                persist: false
            }
        ));
    }

    #[test]
    fn set_with_persist_flag() {
        let args = ["set", "40", "--persist"].into_iter().map(String::from);
        let (_, command) = parse_args(args).unwrap();
        assert!(matches!(
            command,
            Command::Set {
                hz: 40,
                persist: true
            }
        ));
    }

    #[test]
    fn persist_with_get_is_rejected() {
        let args = ["get", "--persist"].into_iter().map(String::from);
        assert!(parse_args(args).is_err());
    }

    #[test]
    fn help_is_recognized() {
        for flag in ["help", "--help", "-h"] {
            let args = [flag].into_iter().map(String::from);
            let (_, command) = parse_args(args).unwrap();
            assert!(matches!(command, Command::Help));
        }
    }

    #[test]
    fn parses_baud_command() {
        let args = ["baud", "921600"].into_iter().map(String::from);
        let (_, command) = parse_args(args).unwrap();
        assert!(matches!(
            command,
            Command::SetBaud {
                baud: 921_600,
                persist: false
            }
        ));
    }

    #[test]
    fn rejects_invalid_baud() {
        let args = ["baud", "100000"].into_iter().map(String::from);
        assert!(parse_args(args).is_err());
    }

    #[test]
    fn builds_reset_commands() {
        // XOR of "VNRST" and "VNRFS"
        assert_eq!(build_command("VNRST"), "$VNRST*4D\r\n");
        assert_eq!(build_command("VNRFS"), "$VNRFS*5F\r\n");
    }

    #[test]
    fn vn_crc16_append_yields_zero() {
        // The property frame validation relies on: CRC over (data + its CRC) == 0.
        let data = [0x01u8, 0x01, 0x01, 0xDE, 0xAD, 0xBE, 0xEF];
        let c = vn_crc16(&data);
        let mut framed = data.to_vec();
        framed.push((c >> 8) as u8); // VN sends CRC MSB first
        framed.push((c & 0xff) as u8);
        assert_eq!(vn_crc16(&framed), 0);
    }

    #[test]
    fn parses_bench_command() {
        let args = ["bench", "--hz", "200", "--secs", "3"]
            .into_iter()
            .map(String::from);
        let (_, command) = parse_args(args).unwrap();
        assert!(matches!(command, Command::Bench { hz: 200, secs: 3 }));
    }

    #[test]
    fn bench_defaults_and_validation() {
        let (_, command) = parse_args(["bench"].into_iter().map(String::from)).unwrap();
        assert!(matches!(command, Command::Bench { hz: 200, secs: 5 }));
        // 150 does not divide 800.
        assert!(parse_args(["bench", "--hz", "150"].into_iter().map(String::from)).is_err());
    }

    #[test]
    fn parses_reset_commands() {
        let (_, reset) = parse_args(["reset"].into_iter().map(String::from)).unwrap();
        assert!(matches!(reset, Command::Reset));
        let (_, factory) = parse_args(["factory-reset"].into_iter().map(String::from)).unwrap();
        assert!(matches!(factory, Command::FactoryReset));
    }

    #[test]
    fn baud_with_persist_flag() {
        let args = ["baud", "921600", "--persist"]
            .into_iter()
            .map(String::from);
        let (_, command) = parse_args(args).unwrap();
        assert!(matches!(
            command,
            Command::SetBaud {
                baud: 921_600,
                persist: true
            }
        ));
    }

    #[test]
    fn read_reply_recovers_reply_after_binary_junk() {
        // Binary bytes (no newline) immediately precede the ASCII echo, as when
        // the binary stream is still flowing during the disable-binary command.
        let mut data = vec![0xFA, 0x01, 0x10, 0x99, 0x00];
        data.extend_from_slice(b"$VNWRG,75,0,4,01,0101*71\r\n");
        let mut cursor = std::io::Cursor::new(data);
        let deadline = Instant::now() + Duration::from_secs(5);
        let got = read_reply(&mut cursor, deadline, |l| l.starts_with("$VNWRG,75")).unwrap();
        assert_eq!(got.as_deref(), Some("$VNWRG,75,0,4,01,0101*71"));
    }

    #[test]
    fn read_reply_honors_deadline_on_newlineless_garbage() {
        // A reader that always yields non-newline bytes must not hang.
        struct Garbage;
        impl Read for Garbage {
            fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
                for b in buf.iter_mut() {
                    *b = b'x';
                }
                Ok(buf.len())
            }
        }
        let deadline = Instant::now() + Duration::from_millis(100);
        let got = read_reply(&mut Garbage, deadline, |_| true).unwrap();
        assert_eq!(got, None);
        assert!(Instant::now() >= deadline);
    }
}
