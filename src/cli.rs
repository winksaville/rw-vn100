//! Command-line surface — argument parsing, the `Command` enum, and help.
//!
//! - `parse_args` turns argv into a `Config` (port/baud) plus a `Command`.
//! - `help_text` renders usage; the ASCII-type helpers map CLI names to
//!   register values.

use crate::proto::*;

/// Frequencies (Hz) the VN-100 accepts for the async data output rate.
/// Authoritative: REFERENCE.md "Register 7" (ICD §3.2.4 Table 3.9; vnsdk AsyncOutputFreq::Adof).
const VALID_RATES: &[u32] = &[1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200];

/// Serial baud rates the VN-100 supports (register 5).
/// Authoritative: REFERENCE.md "Register 5" (ICD §3.2.2 Table 3.3; vnsdk BaudRate::BaudRates).
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

/// What a binary-output config step does (register 75).
///
/// Fields and on/off live in separate verbs so they stay orthogonal
/// — a mask write never toggles streaming, and a streaming toggle
/// never touches the mask:
///
/// - `Fields` — `set-bin-fields=<list>`: set the Common field mask
///   to these fields, leaving streaming on/off unchanged.
/// - `Enable` — `set-bin=on`: enable streaming with the device's
///   current mask (an error if no fields are configured).
/// - `Off` — `set-bin=off`: disable streaming, leaving the mask
///   configured.
pub enum BinSet {
    Fields(Vec<&'static Field>),
    Enable,
    Off,
}

pub enum Command {
    Help,
    Version,
    /// Read the ASCII async preset (register 6).
    GetAscii,
    /// Write the ASCII async preset (register 6); `off` disables it.
    SetAscii {
        preset: u8,
        persist: bool,
    },
    /// Write the ASCII async rate (register 7).
    SetAsciiHz {
        hz: u32,
        persist: bool,
    },
    /// Read the binary output config (register 75).
    GetBin,
    /// Configure the binary output field mask (register 75).
    SetBin {
        action: BinSet,
        persist: bool,
    },
    /// Write the binary output rate as register 75's rateDivisor (`800/hz`).
    SetBinHz {
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
    /// Passively measure whatever the device is already streaming for `secs`
    /// seconds — no device writes. Reports ASCII async line rate, binary
    /// Common-group frame rate, and total wire throughput.
    Bench {
        secs: u64,
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
    s.push_str("rw-vn100 - read/configure a VectorNav VN-100 over serial\n\n");
    s.push_str("Usage: rw-vn100 [--port PORT] [--baud BAUD] <command> [args]\n\n");

    s.push_str("Commands:\n");
    s.push_str("  The output-config verbs each own one register. Adding `--persist`\n");
    s.push_str("  on a `set-*` causes the register to be written to flash.\n\n");
    s.push_str(&help_row(
        "get-ascii",
        &["Read the ASCII async preset (register 6)."],
    ));
    s.push_str(&help_row(
        "set-ascii=<PRESET|off>",
        &[
            "Set the ASCII async preset (register 6).",
            &format!("Presets: {}", ascii_type_names()),
        ],
    ));
    s.push_str(&help_row(
        "set-ascii-hz=<HZ>",
        &["Set the ASCII async rate (register 7)."],
    ));
    s.push_str(&help_row(
        "get-bin",
        &["Read the binary output: port, rate, fields (register 75)."],
    ));
    s.push_str(&help_row(
        "set-bin-fields=<FIELDS>",
        &[
            "Set the binary field mask (register 75), leaving",
            "streaming as-is.",
            &format!("Fields: {}", field_names()),
        ],
    ));
    s.push_str(&help_row(
        "set-bin-hz=<HZ>",
        &["Set the binary rate (register 75 divisor; HZ must divide 800)."],
    ));
    s.push_str(&help_row(
        "set-bin=on|off",
        &["Enable or disable binary streaming (needs fields to enable)."],
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
        "bench [SECS]",
        &[
            "Passively measure whatever is already streaming",
            "(default 5 s) — no device writes. Reports ASCII",
            "line rate, binary frame rate, and wire throughput.",
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

    s.push_str(&format!(
        "\nValid HZ (ASCII async / set-ascii-hz): {VALID_RATES:?}\n"
    ));
    s.push_str("  Fixed in firmware; the device rejects others with a $VNERR.\n");
    s.push_str(&format!("Valid BAUD: {VALID_BAUDS:?}\n\n"));

    s.push_str("Note: a baud change is volatile — the device keeps it across host\n");
    s.push_str("      reconnects, but a power cycle or reset reverts to the flash\n");
    s.push_str("      baud. Persist to keep it.\n\n");

    s.push_str("Examples:\n");
    s.push_str("  rw-vn100 get-ascii\n");
    s.push_str("  rw-vn100 set-ascii-hz=40 --persist\n");
    s.push_str("  rw-vn100 set-bin-fields=time,accel,gyro  # configure fields\n");
    s.push_str("  rw-vn100 set-bin-hz=200                  # set the rate\n");
    s.push_str("  rw-vn100 set-bin=on                      # then enable streaming\n");
    s.push_str("  rw-vn100 rrg 1                      # model number\n");
    s.push_str("  rw-vn100 bench --bin --hz 200 --fields accel,gyro\n");
    s
}

/// The `name version` banner line, e.g. `rw-vn100 0.2.1`.
pub fn version_line() -> String {
    format!("rw-vn100 {}", env!("CARGO_PKG_VERSION"))
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
            _ => positional.push(arg),
        }
    }

    // The config verbs are `key=value` tokens; the older verbs (baud, rrg, wrg,
    // bench, reset) take space-separated positional args. Split the command word
    // on its first '=' so both shapes pass through one match — `value` is `Some`
    // only for the `key=value` form.
    let (verb, value): (Option<&str>, Option<&str>) = match positional.first() {
        Some(c) => match c.split_once('=') {
            Some((v, val)) => (Some(v), Some(val)),
            None => (Some(c.as_str()), None),
        },
        None => (None, None),
    };

    let command = match verb {
        Some("get-ascii") => {
            if persist {
                return Err("--persist applies to a `set-*` verb, not `get-ascii`".into());
            }
            Command::GetAscii
        }
        Some("set-ascii") => {
            let v = value
                .ok_or("set-ascii needs a preset, e.g. `set-ascii=ymr` (or `set-ascii=off`)")?;
            Command::SetAscii {
                preset: parse_ascii_type(v)?,
                persist,
            }
        }
        Some("set-ascii-hz") => {
            let v = value.ok_or("set-ascii-hz needs a rate, e.g. `set-ascii-hz=40`")?;
            let hz: u32 = v.parse().map_err(|_| "ASCII rate must be a number")?;
            if !VALID_RATES.contains(&hz) {
                return Err(format!(
                    "{hz} Hz is not a valid ASCII async rate; choose one of {VALID_RATES:?}"
                ));
            }
            Command::SetAsciiHz { hz, persist }
        }
        Some("get-bin") => {
            if persist {
                return Err("--persist applies to a `set-*` verb, not `get-bin`".into());
            }
            Command::GetBin
        }
        Some("set-bin") => {
            let action = match value {
                Some("on") => BinSet::Enable,
                Some("off") => BinSet::Off,
                _ => {
                    return Err(
                        "set-bin needs =on or =off (set fields with set-bin-fields=<FIELDS>)"
                            .to_string(),
                    );
                }
            };
            Command::SetBin { action, persist }
        }
        Some("set-bin-fields") => {
            let list = value.ok_or(
                "set-bin-fields needs a field list, e.g. `set-bin-fields=time,accel,gyro`",
            )?;
            Command::SetBin {
                action: BinSet::Fields(parse_fields(list)?),
                persist,
            }
        }
        Some("set-bin-hz") => {
            let v = value.ok_or("set-bin-hz needs a rate, e.g. `set-bin-hz=200`")?;
            let hz: u32 = v.parse().map_err(|_| "binary rate must be a number")?;
            if hz == 0 || 800 % hz != 0 {
                return Err(format!(
                    "{hz} Hz invalid for binary; the rate is 800/divisor, so HZ must divide 800 \
                     (e.g. 50, 100, 200, 400)"
                ));
            }
            Command::SetBinHz { hz, persist }
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
            if positional.len() > 2 {
                return Err(
                    "bench takes at most one argument, the measurement duration in seconds \
                     (e.g. `bench 10`); it is passive and configures nothing"
                        .into(),
                );
            }
            let secs = match positional.get(1) {
                Some(s) => s.parse().map_err(|_| {
                    format!("bench duration must be a number of seconds, got `{s}`")
                })?,
                None => 5,
            };
            Command::Bench { secs }
        }
        Some(other) => return Err(format!("unknown command `{other}`")),
        None => {
            return Err(
                "missing command (`get-ascii`, `set-ascii`, `set-ascii-hz`, `get-bin`, \
                 `set-bin-fields`, `set-bin-hz`, `set-bin`, `baud`, `rrg`, `wrg`, \
                 `bench`, `reset`, \
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
    fn parses_set_ascii_hz_and_flags() {
        let args = [
            "--port",
            "/dev/ttyACM0",
            "--baud",
            "921600",
            "set-ascii-hz=40",
        ]
        .into_iter()
        .map(String::from);
        let (config, command) = parse_args(args).unwrap();
        assert_eq!(config.port, "/dev/ttyACM0");
        assert_eq!(config.baud, 921_600);
        assert!(matches!(
            command,
            Command::SetAsciiHz {
                hz: 40,
                persist: false
            }
        ));
    }

    #[test]
    fn rejects_invalid_ascii_rate() {
        assert!(parse_args(["set-ascii-hz=33"].into_iter().map(String::from)).is_err());
    }

    #[test]
    fn parses_set_ascii_preset_and_off() {
        let (_, c) = parse_args(["set-ascii=ymr"].into_iter().map(String::from)).unwrap();
        assert!(matches!(
            c,
            Command::SetAscii {
                preset: 14,
                persist: false
            }
        ));
        let (_, c) = parse_args(["set-ascii=off"].into_iter().map(String::from)).unwrap();
        assert!(matches!(c, Command::SetAscii { preset: 0, .. }));
        // Bare set-ascii is an error: reg 6 has no separate enable bit.
        assert!(parse_args(["set-ascii"].into_iter().map(String::from)).is_err());
    }

    #[test]
    fn parses_get_ascii_with_persist_guard() {
        let (_, c) = parse_args(["get-ascii"].into_iter().map(String::from)).unwrap();
        assert!(matches!(c, Command::GetAscii));
        assert!(parse_args(["get-ascii", "--persist"].into_iter().map(String::from)).is_err());
    }

    #[test]
    fn parses_set_bin_variants() {
        // set-bin-fields sets the mask. The list sorts to bit order
        // (time=0, accel=8) and persists.
        let (_, c) = parse_args(
            ["set-bin-fields=accel,time", "--persist"]
                .into_iter()
                .map(String::from),
        )
        .unwrap();
        match c {
            Command::SetBin {
                action: BinSet::Fields(fields),
                persist,
            } => {
                assert!(persist);
                let names: Vec<&str> = fields.iter().map(|f| f.name).collect();
                assert_eq!(names, vec!["time", "accel"]);
            }
            _ => panic!("expected SetBin Fields"),
        }
        // set-bin=on / =off toggle streaming.
        let (_, c) = parse_args(["set-bin=on"].into_iter().map(String::from)).unwrap();
        assert!(matches!(
            c,
            Command::SetBin {
                action: BinSet::Enable,
                ..
            }
        ));
        let (_, c) = parse_args(["set-bin=off"].into_iter().map(String::from)).unwrap();
        assert!(matches!(
            c,
            Command::SetBin {
                action: BinSet::Off,
                ..
            }
        ));
        // Bare set-bin and a non-on/off value are errors.
        assert!(parse_args(["set-bin"].into_iter().map(String::from)).is_err());
        assert!(parse_args(["set-bin=bogus"].into_iter().map(String::from)).is_err());
        // Unknown field, and bare set-bin-fields, are errors.
        assert!(parse_args(["set-bin-fields=bogus"].into_iter().map(String::from)).is_err());
        assert!(parse_args(["set-bin-fields"].into_iter().map(String::from)).is_err());
    }

    #[test]
    fn parses_set_bin_hz_and_validates_divisor() {
        let (_, c) = parse_args(["set-bin-hz=200"].into_iter().map(String::from)).unwrap();
        assert!(matches!(
            c,
            Command::SetBinHz {
                hz: 200,
                persist: false
            }
        ));
        // 150 does not divide 800.
        assert!(parse_args(["set-bin-hz=150"].into_iter().map(String::from)).is_err());
        // Bare set-bin-hz needs a value.
        assert!(parse_args(["set-bin-hz"].into_iter().map(String::from)).is_err());
    }

    #[test]
    fn parses_get_bin() {
        let (_, c) = parse_args(["get-bin"].into_iter().map(String::from)).unwrap();
        assert!(matches!(c, Command::GetBin));
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
    fn parses_bench_passive() {
        // Bare bench defaults to 5 s.
        let (_, c) = parse_args(["bench"].into_iter().map(String::from)).unwrap();
        assert!(matches!(c, Command::Bench { secs: 5 }));
        // Positional SECS.
        let (_, c) = parse_args(["bench", "10"].into_iter().map(String::from)).unwrap();
        assert!(matches!(c, Command::Bench { secs: 10 }));
        // Non-numeric duration is rejected.
        assert!(parse_args(["bench", "abc"].into_iter().map(String::from)).is_err());
        // bench takes at most one argument — it is passive and configures nothing.
        assert!(parse_args(["bench", "1", "2"].into_iter().map(String::from)).is_err());
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
    fn parses_ascii_type() {
        assert_eq!(parse_ascii_type("vnymr").unwrap(), 14); // ICD §3.2.3 / Ador::YMR
        assert_eq!(parse_ascii_type("YMR").unwrap(), 14);
        assert_eq!(parse_ascii_type("qtn").unwrap(), 2);
        assert!(parse_ascii_type("bogus").is_err());
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
