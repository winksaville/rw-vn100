//! Command-line surface — argument parsing, the `Command` enum, and help.
//!
//! - `parse_args` turns argv into a `Config` (port/baud) plus a `Command`.
//! - `help_text` renders usage; the ASCII-type and serial-port helpers map
//!   CLI names to register values.

use crate::proto::*;

/// Frequencies (Hz) the VN-100 accepts for the async data output rate.
/// Authoritative: REFERENCE.md "Register 7" (ICD Reg 7; vnsdk AsyncOutputFreq::Adof).
const VALID_RATES: &[u32] = &[1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200];

/// Serial baud rates the VN-100 supports (register 5).
/// Authoritative: REFERENCE.md "Register 5" (ICD Reg 5; vnsdk BaudRate::BaudRates).
const VALID_BAUDS: &[u32] = &[
    9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600,
];

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
pub fn ascii_type_name(value: u8) -> String {
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

pub struct Config {
    pub port: String,
    pub baud: u32,
}

pub enum Command {
    Help,
    Version,
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

pub fn help_text() -> String {
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
    s.push_str(&help_row(
        "--version | -V",
        &["Print the version and exit."],
    ));

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

/// The `name version` banner line, e.g. `rdwr_vn100 0.2.1`.
pub fn version_line() -> String {
    format!("rdwr_vn100 {}", env!("CARGO_PKG_VERSION"))
}

/// Parse CLI args into a connection config and a command.
pub fn parse_args<I: Iterator<Item = String>>(args: I) -> Result<(Config, Command), String> {
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
    if args
        .iter()
        .any(|a| matches!(a.as_str(), "--version" | "-V"))
    {
        return Ok((
            Config {
                port: String::new(),
                baud: 0,
            },
            Command::Version,
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
            );
        }
    };

    Ok((Config { port, baud }, command))
}

#[cfg(test)]
mod tests {
    use super::*;

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
        assert!(
            parse_args(
                ["bench", "--serial-port", "2"]
                    .into_iter()
                    .map(String::from)
            )
            .is_err()
        );
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
        assert!(
            parse_args(
                ["bench", "--bin", "--type", "ymr"]
                    .into_iter()
                    .map(String::from)
            )
            .is_err()
        );
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
        assert!(
            parse_args(
                ["bench", "--bin", "--hz", "150"]
                    .into_iter()
                    .map(String::from)
            )
            .is_err()
        );
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
}
