//! `rdwr_vn100` — read and configure a VectorNav VN-100 IMU over serial.
//!
//! The VN-100 speaks an ASCII protocol over a serial port. Each command is
//!
//!     $<payload>*XX\r\n
//!
//! where `XX` is the 8-bit XOR checksum of every character of `<payload>`
//! (i.e. everything between `$` and `*`). Replies echo the same form; binary
//! outputs (register 75) use a packed frame ending in a 16-bit CRC instead.
//!
//! Commands implemented here:
//!   get-hz / set-hz       read/write the async output rate    (register 7)
//!   baud                  change the serial baud rate          (register 5)
//!   rrg / wrg             generic read/write of any register   ($VNRRG / $VNWRG)
//!   bench                 configure an output (ASCII async by default, or a
//!                         binary output with --bin) and measure the achieved
//!                         rate, to see what fits a given baud
//!   reset / factory-reset reboot / restore defaults            ($VNRST / $VNRFS)
//!
//! Key VN messages:
//!   $VNRRG,<id>*XX          -> $VNRRG,<id>,<f1>,...*YY   (read register)
//!   $VNWRG,<id>,<f1>,...*XX  -> echo                     (write register)
//!   $VNWNV*XX               -> $VNWNV*YY                 (save all to flash)
//!   $VNERR,<code>*XX                                     (error; see error_description)
//!
//! The async output rate (register 7) is one of a fixed firmware set
//! (`VALID_RATES`); a value that's out of range — or too much data for the
//! current baud — returns a `$VNERR` (0x0C = insufficient baud rate).
//!
//! Register/enum/table values are cited to the ICD and vnsdk in REFERENCE.md.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

/// Frequencies (Hz) the VN-100 accepts for the async data output rate.
/// Authoritative: REFERENCE.md "Register 7" (ICD Reg 7; vnsdk AsyncOutputFreq::Adof).
const VALID_RATES: &[u32] = &[1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200];

/// Serial baud rates the VN-100 supports (register 5).
/// Authoritative: REFERENCE.md "Register 5" (ICD Reg 5; vnsdk BaudRate::BaudRates).
const VALID_BAUDS: &[u32] = &[
    9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600,
];

/// A selectable binary-output field (all from the "Common" group, group 1):
/// CLI name, the group-1 bit it occupies, and its on-wire byte size.
struct Field {
    name: &'static str,
    bit: u8,
    size: usize,
}

/// The `--fields` vocabulary (Common group only — keeps the frame to one group).
/// Authoritative: REFERENCE.md "Common Group" (ICD §2.2 Table 2.3).
const FIELDS: &[Field] = &[
    Field {
        name: "time",
        bit: 0,
        size: 8,
    }, // TimeStartup, u64 ns
    Field {
        name: "ypr",
        bit: 3,
        size: 12,
    }, // YawPitchRoll, 3×f32 deg
    Field {
        name: "quat",
        bit: 4,
        size: 16,
    }, // Quaternion, 4×f32
    Field {
        name: "gyro",
        bit: 5,
        size: 12,
    }, // AngularRate, 3×f32 rad/s
    Field {
        name: "accel",
        bit: 8,
        size: 12,
    }, // Accel, 3×f32 m/s^2
    Field {
        name: "imu",
        bit: 9,
        size: 24,
    }, // uncomp Accel+Gyro, 6×f32
    Field {
        name: "magpres",
        bit: 10,
        size: 20,
    }, // Mag(3×f32)+Temp+Pres
];

fn lookup_field(name: &str) -> Option<&'static Field> {
    FIELDS.iter().find(|f| f.name == name)
}

fn field_names() -> String {
    FIELDS.iter().map(|f| f.name).collect::<Vec<_>>().join(", ")
}

/// Parse a comma-separated `--fields` list into Common-group fields, ordered by
/// bit (the order the device emits them), de-duplicated.
fn parse_fields(list: &str) -> Result<Vec<&'static Field>, String> {
    let mut out: Vec<&'static Field> = Vec::new();
    for name in list.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        let f = lookup_field(name)
            .ok_or_else(|| format!("unknown field `{name}`; choose from {}", field_names()))?;
        if !out.iter().any(|g| g.name == f.name) {
            out.push(f);
        }
    }
    if out.is_empty() {
        return Err("--fields needs at least one field".into());
    }
    out.sort_by_key(|f| f.bit);
    Ok(out)
}

/// Default binary field set: timestamp + acceleration.
fn default_fields() -> Vec<&'static Field> {
    vec![
        lookup_field("time").unwrap(),
        lookup_field("accel").unwrap(),
    ]
}

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
/// Authoritative: REFERENCE.md "Error responses" (ICD §1; vnsdk Errors.hpp Error).
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

/// Parse register 6 (Async Data Output Type / ADOR) value from a reply.
fn parse_reg06(line: &str) -> Option<u8> {
    let body = line
        .strip_prefix("$VNRRG,06,")
        .or_else(|| line.strip_prefix("$VNWRG,06,"))?;
    body.split('*').next()?.trim().parse().ok()
}

/// ASCII async message presets (register 6 ADOR): CLI name -> register value.
/// Authoritative: REFERENCE.md "Register 6" (ICD §3.2.3 Table 3.6;
/// vnsdk AsyncOutputType::Ador).
const ASCII_TYPES: &[(&str, u8)] = &[
    ("off", 0),
    ("ypr", 1),
    ("qtn", 2),
    ("qmr", 8),
    ("mag", 10),
    ("acc", 11),
    ("gyr", 12),
    ("mar", 13),
    ("ymr", 14), // the factory default
    ("yba", 16),
    ("yia", 17),
    ("imu", 19),
    ("dtv", 30),
    ("hve", 34),
];

fn ascii_type_names() -> String {
    ASCII_TYPES
        .iter()
        .map(|(n, _)| *n)
        .collect::<Vec<_>>()
        .join(", ")
}

/// Resolve a `--type` name (case-insensitive, optional `vn` prefix) to its ADOR.
fn parse_ascii_type(s: &str) -> Result<u8, String> {
    let lower = s.trim().to_lowercase();
    let key = lower.strip_prefix("vn").unwrap_or(&lower);
    ASCII_TYPES
        .iter()
        .find(|(n, _)| *n == key)
        .map(|(_, v)| *v)
        .ok_or_else(|| {
            format!(
                "unknown ASCII type `{s}`; choose from {}",
                ascii_type_names()
            )
        })
}

/// Parse the `--serial-port` value into a register-75 `asyncMode` code.
///
/// - `1` / `2` target one of the VN-100's two UARTs; `both` (or `3`) targets
///   both. The default elsewhere is `2`, the RPi5 TTL header (the flight
///   target); the RS-232 bench is port 1. Selecting a port also subjects it to
///   the register-75 fit check, so `both` is avoided as a default — a port left
///   at a low baud would veto a frame the connected port could take.
fn parse_serial_port(s: &str) -> Result<u8, String> {
    match s.trim().to_lowercase().as_str() {
        "1" => Ok(1),
        "2" => Ok(2),
        "3" | "both" => Ok(3),
        other => Err(format!(
            "invalid --serial-port `{other}`; choose 1, 2, or both"
        )),
    }
}

/// Display name for an ADOR value, e.g. 8 -> "VNYMR".
fn ascii_type_name(value: u8) -> String {
    ASCII_TYPES
        .iter()
        .find(|(_, v)| *v == value)
        .map(|(n, _)| {
            if *n == "off" {
                "off".to_string()
            } else {
                format!("VN{}", n.to_uppercase())
            }
        })
        .unwrap_or_else(|| format!("ADOR {value}"))
}

struct Config {
    port: String,
    baud: u32,
}

enum Command {
    Help,
    GetHz,
    SetHz {
        hz: u32,
        persist: bool,
    },
    SetBaud {
        baud: u32,
        persist: bool,
    },
    Reset,
    FactoryReset,
    /// Read any register (generic ASCII Read Register).
    Rrg {
        id: u8,
    },
    /// Write any register (generic ASCII Write Register).
    Wrg {
        id: u8,
        params: Vec<String>,
    },
    /// Configure an output (ASCII async by default, or binary with `--bin`) and
    /// measure the achieved rate, then restore prior state.
    ///
    /// - `serial_port` is the register-75 `asyncMode` (binary only): 1 or 2 for
    ///   one of the VN-100's two UARTs, 3 for both. Ignored for the ASCII bench.
    Bench {
        binary: bool,
        hz: u32,
        secs: u64,
        fields: Vec<&'static Field>,
        serial_port: u8,
        ascii_type: Option<u8>,
    },
}

/// Format one help row: a `label` and one-or-more wrapped description lines,
/// aligned to a common description column. If the label is too wide, the
/// description starts on the next line.
fn help_row(label: &str, desc: &[&str]) -> String {
    const COL: usize = 24; // label column width; description starts after it
    let mut out = String::new();
    if label.len() <= COL {
        out.push_str(&format!("  {label:<COL$}{}\n", desc[0]));
    } else {
        out.push_str(&format!("  {label}\n"));
        out.push_str(&format!("  {:<COL$}{}\n", "", desc[0]));
    }
    for cont in &desc[1..] {
        out.push_str(&format!("  {:<COL$}{}\n", "", cont));
    }
    out
}

fn help_text() -> String {
    let mut s = String::new();
    s.push_str("rdwr_vn100 - read/configure a VectorNav VN-100 over serial\n\n");
    s.push_str("Usage: rdwr_vn100 [--port PORT] [--baud BAUD] <command> [args]\n\n");

    s.push_str("Commands:\n");
    s.push_str(&help_row(
        "get-hz",
        &["Read the async output rate (register 7)."],
    ));
    s.push_str(&help_row(
        "set-hz <HZ> [--persist]",
        &["Write the async output rate (validated)."],
    ));
    s.push_str(&help_row(
        "baud <NEW> [--persist]",
        &[
            "Change serial baud (register 5); switch this",
            "connection to it and verify, without closing",
            "the port.",
        ],
    ));
    s.push_str(&help_row(
        "rrg <ID>",
        &["Read any register; print its fields."],
    ));
    s.push_str(&help_row(
        "wrg <ID> <P1> [P2...]",
        &[
            "Write any register. Sharp tool: e.g.",
            "`wrg 5 921600` skips the safe baud switch —",
            "use `baud` instead.",
        ],
    ));
    s.push_str(&help_row(
        "bench [--bin] [--hz HZ] [--secs S] [--fields LIST]",
        &[
            "Configure an output and measure the achieved",
            "rate, then restore. ASCII async by default;",
            "--bin selects a binary output (register 75).",
        ],
    ));
    s.push_str(&help_row(
        "reset | factory-reset",
        &["Reboot / restore-to-defaults ($VNRST / $VNRFS)."],
    ));
    s.push_str(&help_row("help | --help | -h", &["Show this help."]));

    s.push_str("\nBench options:\n");
    s.push_str(&help_row(
        "--bin",
        &["Binary output (register 75) instead of ASCII async."],
    ));
    s.push_str(&help_row(
        "--hz HZ",
        &[
            "Output rate (default 40). ASCII: a valid HZ below.",
            "Binary: must divide 800 (up to 800; link may cap lower).",
        ],
    ));
    s.push_str(&help_row(
        "--secs S",
        &["Measurement duration in seconds (default 5)."],
    ));
    s.push_str(&help_row(
        "--fields L",
        &[
            "Binary only: comma-separated fields (default time,accel).",
            &format!("Choices: {}", field_names()),
        ],
    ));
    s.push_str(&help_row(
        "--serial-port P",
        &[
            "Binary only: VN-100 UART(s) to stream on — 1, 2,",
            "or both (default 2, the RPi5 TTL header). The",
            "RS-232 bench is port 1. `both` also makes a port",
            "left at a low baud veto the fit check.",
        ],
    ));
    s.push_str(&help_row(
        "--type NAME",
        &[
            "ASCII only: set the message preset (register 6) first.",
            &format!("Choices: {}", ascii_type_names()),
        ],
    ));

    s.push_str("\nGlobal options:\n");
    s.push_str(&help_row(
        "--port PORT",
        &[
            "Serial device (default: /dev/ttyAMA0, the RPi5",
            "header UART). Use /dev/ttyUSB0 for a USB adapter.",
        ],
    ));
    s.push_str(&help_row(
        "--baud BAUD",
        &[
            "Baud to talk to the device NOW (default 115200);",
            "must match the device's CURRENT rate.",
        ],
    ));
    s.push_str(&help_row(
        "--persist",
        &["Save to flash so it survives a power cycle (set-hz, baud)."],
    ));

    s.push_str(&format!("\nValid HZ (ASCII / set-hz): {VALID_RATES:?}\n"));
    s.push_str("  Fixed in firmware; the device rejects others with a $VNERR.\n");
    s.push_str(&format!("Valid BAUD: {VALID_BAUDS:?}\n\n"));

    s.push_str("Note: a baud change is volatile — the device keeps it across host\n");
    s.push_str("      reconnects, but a power cycle or reset reverts to the flash\n");
    s.push_str("      baud. Persist to keep it.\n\n");

    s.push_str("Examples:\n");
    s.push_str("  rdwr_vn100 get-hz\n");
    s.push_str("  rdwr_vn100 set-hz 40 --persist\n");
    s.push_str("  rdwr_vn100 rrg 1                      # model number\n");
    s.push_str("  rdwr_vn100 bench --bin --hz 200 --fields accel,gyro\n");
    s.push_str("  rdwr_vn100 bench --hz 50              # ASCII async at 50 Hz\n");
    s
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

    let mut port = "/dev/ttyAMA0".to_string();
    let mut baud = 115_200u32;
    let mut persist = false;
    let mut binary = false;
    let mut hz: Option<u32> = None;
    let mut secs: Option<u64> = None;
    let mut fields_arg: Option<String> = None;
    let mut serial_port_arg: Option<String> = None;
    let mut type_arg: Option<String> = None;
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
            "--bin" => binary = true,
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
            "--fields" => fields_arg = Some(args.next().ok_or("--fields requires a value")?),
            "--serial-port" => {
                serial_port_arg = Some(args.next().ok_or("--serial-port requires a value")?)
            }
            "--type" => type_arg = Some(args.next().ok_or("--type requires a value")?),
            _ => positional.push(arg),
        }
    }

    let command = match positional.first().map(String::as_str) {
        Some("get-hz") => {
            if persist {
                return Err("--persist only applies to `set-hz`".into());
            }
            Command::GetHz
        }
        Some("set-hz") => {
            let hz: u32 = positional
                .get(1)
                .ok_or("set-hz requires a frequency, e.g. `set-hz 40`")?
                .parse()
                .map_err(|_| "frequency must be a number")?;
            if !VALID_RATES.contains(&hz) {
                return Err(format!(
                    "{hz} Hz is not valid; choose one of {VALID_RATES:?}"
                ));
            }
            Command::SetHz { hz, persist }
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
        Some("rrg") => {
            let id: u8 = positional
                .get(1)
                .ok_or("rrg requires a register id, e.g. `rrg 1`")?
                .parse()
                .map_err(|_| "register id must be 0-255")?;
            Command::Rrg { id }
        }
        Some("wrg") => {
            let id: u8 = positional
                .get(1)
                .ok_or("wrg requires a register id, e.g. `wrg 7 40`")?
                .parse()
                .map_err(|_| "register id must be 0-255")?;
            let params: Vec<String> = positional.iter().skip(2).cloned().collect();
            if params.is_empty() {
                return Err("wrg requires at least one value, e.g. `wrg 7 40`".into());
            }
            Command::Wrg { id, params }
        }
        Some("reset") => Command::Reset,
        Some("factory-reset") => Command::FactoryReset,
        Some("bench") => {
            let hz = hz.unwrap_or(40);
            let secs = secs.unwrap_or(5);
            if binary {
                if type_arg.is_some() {
                    return Err(
                        "--type only applies to the ASCII bench (binary picks data with --fields)"
                            .into(),
                    );
                }
                if hz == 0 || 800 % hz != 0 {
                    return Err(format!(
                        "--hz {hz} invalid for --bin; the binary rate is 800/divisor, so HZ must \
                         divide 800 (e.g. 50, 100, 200, 400)"
                    ));
                }
                let fields = match &fields_arg {
                    Some(list) => parse_fields(list)?,
                    None => default_fields(),
                };
                // Default to port 2, the RPi5 TTL header (the flight target).
                // Not `both`: selecting a port also subjects it to the reg-75
                // fit check, so once port 2 is raised to a high baud with port 1
                // left low, `both` would let port 1 veto a frame port 2 can take.
                let serial_port = match &serial_port_arg {
                    Some(s) => parse_serial_port(s)?,
                    None => 2,
                };
                Command::Bench {
                    binary: true,
                    hz,
                    secs,
                    fields,
                    serial_port,
                    ascii_type: None,
                }
            } else {
                if fields_arg.is_some() {
                    return Err(
                        "--fields only applies with --bin (ASCII async uses preset messages, \
                         not arbitrary fields)"
                            .into(),
                    );
                }
                if serial_port_arg.is_some() {
                    return Err(
                        "--serial-port only applies with --bin (register 75); the ASCII async \
                         output targets the connected port automatically"
                            .into(),
                    );
                }
                let ascii_type = match &type_arg {
                    Some(t) => Some(parse_ascii_type(t)?),
                    None => None,
                };
                if !VALID_RATES.contains(&hz) {
                    return Err(format!(
                        "--hz {hz} not valid for the ASCII async output; choose one of \
                         {VALID_RATES:?} (or use --bin)"
                    ));
                }
                Command::Bench {
                    binary: false,
                    hz,
                    secs,
                    fields: Vec::new(),
                    serial_port: 0, // unused for ASCII
                    ascii_type,
                }
            }
        }
        Some(other) => return Err(format!("unknown command `{other}`")),
        None => {
            return Err(
                "missing command (`get-hz`, `set-hz`, `baud`, `rrg`, `wrg`, `bench`, `reset`, \
                 `factory-reset`, or `help`)"
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
/// produces 0. Authoritative: REFERENCE.md "Framing & checksums" (ICD §1.4).
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
                continue
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
                continue
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
fn bench_binary<S: Read + Write>(
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

    // Configure Binary Output 1 (reg 75) on the chosen serial port(s). A $VNERR
    // here means the chosen fields+rate don't fit the current baud — nothing
    // else has changed.
    let cfg = format!("VNWRG,75,{serial_port},{divisor},01,{mask:04X}");
    transact_retry(
        port,
        &build_command(&cfg),
        5,
        |l| l.starts_with("$VNWRG,75"),
        "device did not accept the binary config (a $VNERR means it won't fit at this baud)",
    )?;
    println!(
        "Configured binary output: Common{names:?} @ {hz} Hz (divisor {divisor}, {frame_len} B/frame)."
    );

    // Silence the ASCII async output so we measure ONLY the binary stream.
    transact_retry(
        port,
        &build_command("VNWRG,07,0"),
        5,
        |l| parse_reg07(l).is_some(),
        "could not disable ASCII async output",
    )?;

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
fn bench_ascii<S: Read + Write>(
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

/// Print the fields of a `$VN(R|W)RG,<id>,<f1>,<f2>...*XX` reply.
fn print_reg_fields(reply: &str) {
    let body = reply.trim_start_matches('$');
    let body = body.split('*').next().unwrap_or(body);
    let parts: Vec<&str> = body.split(',').collect();
    match parts.as_slice() {
        [_, id, fields @ ..] if !fields.is_empty() => {
            println!("register {id}: {fields:?}");
        }
        [_, id] => println!("register {id}: (no fields)"),
        _ => println!("(unrecognized reply)"),
    }
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

        Command::GetHz => {
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

        Command::SetHz { hz, persist } => {
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

        Command::Rrg { id } => {
            let reply = transact_retry(
                &mut port,
                &build_command(&format!("VNRRG,{id:02}")),
                5,
                |l| l.starts_with("$VNRRG,"),
                &no_reply,
            )?;
            println!("RX: {reply}");
            print_reg_fields(&reply);
        }

        Command::Wrg { id, params } => {
            let payload = format!("VNWRG,{id:02},{}", params.join(","));
            let reply = transact_retry(
                &mut port,
                &build_command(&payload),
                5,
                |l| l.starts_with("$VNWRG,"),
                &no_reply,
            )?;
            println!("RX: {reply}");
            print_reg_fields(&reply);
        }

        Command::Bench {
            binary,
            hz,
            secs,
            fields,
            serial_port,
            ascii_type,
        } => {
            if binary {
                bench_binary(&mut port, config.baud, hz, secs, &fields, serial_port)?;
            } else {
                bench_ascii(&mut port, config.baud, hz, secs, ascii_type)?;
            }
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
        let args = ["set-hz", "33"].into_iter().map(String::from);
        assert!(parse_args(args).is_err());
    }

    #[test]
    fn parses_flags_and_set_hz_command() {
        let args = ["--port", "/dev/ttyACM0", "--baud", "921600", "set-hz", "40"]
            .into_iter()
            .map(String::from);
        let (config, command) = parse_args(args).unwrap();
        assert_eq!(config.port, "/dev/ttyACM0");
        assert_eq!(config.baud, 921_600);
        assert!(matches!(
            command,
            Command::SetHz {
                hz: 40,
                persist: false
            }
        ));
    }

    #[test]
    fn set_hz_with_persist_flag() {
        let args = ["set-hz", "40", "--persist"].into_iter().map(String::from);
        let (_, command) = parse_args(args).unwrap();
        assert!(matches!(
            command,
            Command::SetHz {
                hz: 40,
                persist: true
            }
        ));
    }

    #[test]
    fn persist_with_get_hz_is_rejected() {
        let args = ["get-hz", "--persist"].into_iter().map(String::from);
        assert!(parse_args(args).is_err());
    }

    #[test]
    fn parses_rrg_and_wrg() {
        let (_, c) = parse_args(["rrg", "1"].into_iter().map(String::from)).unwrap();
        assert!(matches!(c, Command::Rrg { id: 1 }));

        let (_, c) = parse_args(["wrg", "7", "40"].into_iter().map(String::from)).unwrap();
        match c {
            Command::Wrg { id, params } => {
                assert_eq!(id, 7);
                assert_eq!(params, vec!["40".to_string()]);
            }
            _ => panic!("expected Wrg"),
        }

        // wrg needs at least one value
        assert!(parse_args(["wrg", "7"].into_iter().map(String::from)).is_err());
    }

    #[test]
    fn fields_parse_orders_by_bit_and_dedups() {
        let f = parse_fields("accel,time,accel").unwrap();
        let names: Vec<&str> = f.iter().map(|x| x.name).collect();
        assert_eq!(names, vec!["time", "accel"]); // bit-ordered + de-duplicated
        let mask: u16 = f.iter().fold(0, |m, x| m | (1u16 << x.bit));
        assert_eq!(mask, 0x0101); // time bit0 + accel bit8 — matches the known config
        assert!(parse_fields("bogus").is_err());
    }

    #[test]
    fn parses_bench_bin_with_fields() {
        let args = [
            "bench",
            "--bin",
            "--hz",
            "200",
            "--fields",
            "time,accel,gyro",
        ]
        .into_iter()
        .map(String::from);
        let (_, c) = parse_args(args).unwrap();
        match c {
            Command::Bench {
                binary,
                hz,
                fields,
                serial_port,
                ..
            } => {
                assert!(binary);
                assert_eq!(hz, 200);
                assert_eq!(serial_port, 2); // default port 2 (RPi5 TTL header)
                let names: Vec<&str> = fields.iter().map(|f| f.name).collect();
                assert_eq!(names, vec!["time", "gyro", "accel"]); // bits 0, 5, 8
            }
            _ => panic!("expected Bench"),
        }
        // --fields requires --bin
        assert!(parse_args(["bench", "--fields", "time"].into_iter().map(String::from)).is_err());
    }

    #[test]
    fn parses_serial_port() {
        assert_eq!(parse_serial_port("1").unwrap(), 1);
        assert_eq!(parse_serial_port("2").unwrap(), 2);
        assert_eq!(parse_serial_port("both").unwrap(), 3);
        assert_eq!(parse_serial_port("3").unwrap(), 3);
        assert!(parse_serial_port("0").is_err());

        // --serial-port flows through to the binary bench config.
        let (_, c) = parse_args(
            ["bench", "--bin", "--serial-port", "2"]
                .into_iter()
                .map(String::from),
        )
        .unwrap();
        match c {
            Command::Bench { serial_port, .. } => assert_eq!(serial_port, 2),
            _ => panic!("expected Bench"),
        }

        // --serial-port requires --bin (ASCII targets the connected port).
        assert!(parse_args(
            ["bench", "--serial-port", "2"]
                .into_iter()
                .map(String::from)
        )
        .is_err());
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
    fn parses_bench_ascii_command() {
        let args = ["bench", "--hz", "50", "--secs", "3"]
            .into_iter()
            .map(String::from);
        let (_, command) = parse_args(args).unwrap();
        match command {
            Command::Bench {
                binary,
                hz,
                secs,
                fields,
                serial_port,
                ascii_type,
            } => {
                assert!(!binary); // ASCII is the default
                assert_eq!(hz, 50);
                assert_eq!(secs, 3);
                assert!(fields.is_empty());
                assert_eq!(serial_port, 0); // unused for ASCII
                assert_eq!(ascii_type, None);
            }
            _ => panic!("expected Bench"),
        }
    }

    #[test]
    fn parses_ascii_type() {
        assert_eq!(parse_ascii_type("vnymr").unwrap(), 14); // ICD §3.2.3 / Ador::YMR
        assert_eq!(parse_ascii_type("YMR").unwrap(), 14);
        assert_eq!(parse_ascii_type("qtn").unwrap(), 2);
        assert!(parse_ascii_type("bogus").is_err());

        let (_, c) =
            parse_args(["bench", "--type", "vnqtn"].into_iter().map(String::from)).unwrap();
        match c {
            Command::Bench {
                binary, ascii_type, ..
            } => {
                assert!(!binary);
                assert_eq!(ascii_type, Some(2));
            }
            _ => panic!("expected Bench"),
        }

        // --type with --bin is rejected.
        assert!(parse_args(
            ["bench", "--bin", "--type", "ymr"]
                .into_iter()
                .map(String::from)
        )
        .is_err());
    }

    #[test]
    fn bench_defaults_and_validation() {
        // Bare bench: ASCII, 40 Hz, 5 s.
        let (_, command) = parse_args(["bench"].into_iter().map(String::from)).unwrap();
        match command {
            Command::Bench {
                binary, hz, secs, ..
            } => {
                assert!(!binary);
                assert_eq!(hz, 40);
                assert_eq!(secs, 5);
            }
            _ => panic!("expected Bench"),
        }
        // 150 is not a valid ASCII async rate.
        assert!(parse_args(["bench", "--hz", "150"].into_iter().map(String::from)).is_err());
        // 150 does not divide 800 for binary either.
        assert!(parse_args(
            ["bench", "--bin", "--hz", "150"]
                .into_iter()
                .map(String::from)
        )
        .is_err());
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
