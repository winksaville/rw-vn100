//! `rw-vn100` — read and configure a VectorNav VN-100 IMU over serial.
//!
//! The VN-100 speaks an ASCII protocol over a serial port. Each command is
//!
//! ```text
//! $<payload>*XX\r\n
//! ```
//!
//! where `XX` is the 8-bit XOR checksum of every character of `<payload>`
//! (i.e. everything between `$` and `*`). Replies echo the same form; binary
//! outputs (register 75) use a packed frame ending in a 16-bit CRC instead.
//!
//! Commands implemented here:
//!   get-ascii / set-ascii read/write the ASCII async preset    (register 6)
//!   set-ascii-hz          write the ASCII async rate           (register 7)
//!   get-bin / set-bin     read/configure the binary output     (register 75)
//!   set-bin-hz            write the binary rate (reg-75 divisor, 800/hz)
//!   baud                  change the serial baud rate          (register 5)
//!   rrg / wrg             generic read/write of any register   ($VNRRG / $VNWRG)
//!   bench                 (legacy) configure an output and measure the achieved
//!                         rate; superseded by passive bench in a later step
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

use std::time::Duration;

mod bench;
mod cli;
mod proto;
mod transact;
use bench::*;
use cli::*;
use proto::*;
use transact::*;

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

/// Write the device's current settings to non-volatile memory (`$VNWNV`) so
/// they survive a power cycle. Shared by the `set-*` verbs' `--persist`.
fn save_to_flash<S: std::io::Read + std::io::Write>(
    port: &mut S,
    no_reply: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let confirm = transact_retry(
        port,
        &build_command("VNWNV"),
        5,
        |l| l.starts_with("$VNWNV"),
        no_reply,
    )?;
    println!("RX: {confirm}");
    println!("Settings written to non-volatile memory.");
    Ok(())
}

/// Print a one-line summary of a binary-output (register 75) config.
///
/// - State is the streaming port (`off`, `port N`, or both).
/// - Rate is `800 / divisor`; the Common fields come from the mask.
fn report_bin_config(r: &Reg75) {
    let state = match r.async_mode {
        0 => "off".to_string(),
        3 => "on (both ports)".to_string(),
        p => format!("on (port {p})"),
    };
    let rate = if r.divisor > 0 {
        format!("{} Hz", 800 / r.divisor as u32)
    } else {
        "rate unset".to_string()
    };
    let names: Vec<&str> = fields_from_mask(r.mask).iter().map(|f| f.name).collect();
    println!(
        "Binary output: {state}, {rate} (divisor {}), Common{names:?}.",
        r.divisor
    );
}

/// Parse args, open the serial port, and dispatch the requested command.
pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let (config, command) = match parse_args(std::env::args().skip(1)) {
        Ok(parsed) => parsed,
        Err(e) => {
            // Print only the error (plus a pointer) — not the whole help, which
            // on a small terminal scrolls the error itself off the top.
            eprintln!("error: {e}");
            eprintln!("run `rw-vn100 --help` for usage");
            std::process::exit(2);
        }
    };

    if let Command::Help = command {
        print!("{}", help_text());
        return Ok(());
    }

    if let Command::Version = command {
        println!("{}", version_line());
        return Ok(());
    }

    // Stamp a bench run with the tool version as its first line, so a captured
    // bench log records which build produced the numbers.
    if let Command::Bench { .. } = command {
        println!("{}", version_line());
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
        Command::Help | Command::Version => unreachable!("handled above"),

        Command::GetAscii => {
            let reply = transact_retry(
                &mut port,
                &build_command("VNRRG,06"),
                5,
                |l| parse_reg06(l).is_some(),
                &no_reply,
            )?;
            println!("RX: {reply}");
            let v = parse_reg06(&reply).unwrap(); // OK: predicate guaranteed parse_reg06 Some
            println!("ASCII async preset: {} (ADOR {v})", ascii_type_name(v));
        }

        Command::SetAscii { preset, persist } => {
            let reply = transact_retry(
                &mut port,
                &build_command(&format!("VNWRG,06,{preset}")),
                5,
                |l| l.starts_with("$VNWRG,06"),
                &no_reply,
            )?;
            println!("RX: {reply}");
            println!(
                "ASCII async preset set to {} (ADOR {preset}).",
                ascii_type_name(preset)
            );
            if persist {
                save_to_flash(&mut port, &no_reply)?;
            }
        }

        Command::SetAsciiHz { hz, persist } => {
            let reply = transact_retry(
                &mut port,
                &build_command(&format!("VNWRG,07,{hz}")),
                5,
                |l| parse_reg07(l).is_some(),
                &no_reply,
            )?;
            println!("RX: {reply}");
            println!("ASCII async rate: {} Hz", parse_reg07(&reply).unwrap()); // OK: predicate ensured Some
            if persist {
                save_to_flash(&mut port, &no_reply)?;
            }
        }

        Command::GetBin => {
            let reply = transact_retry(
                &mut port,
                &build_command("VNRRG,75"),
                5,
                |l| parse_reg75(l).is_some(),
                &no_reply,
            )?;
            println!("RX: {reply}");
            report_bin_config(&parse_reg75(&reply).unwrap()); // OK: predicate ensured Some
        }

        Command::SetBin { action, persist } => {
            // Read-modify-write reg 75: preserve the rateDivisor, change only the
            // streaming port and/or the Common field mask per the action.
            let cur = transact_retry(
                &mut port,
                &build_command("VNRRG,75"),
                5,
                |l| parse_reg75(l).is_some(),
                &no_reply,
            )?;
            let r = parse_reg75(&cur).unwrap(); // OK: predicate ensured Some
            // Enabling defaults to port 2 (the RPi5 TTL header) when binary is
            // currently off; otherwise it keeps whichever port is already set.
            let on_port = if r.async_mode == 0 { 2 } else { r.async_mode };
            let (mode, mask) = match action {
                BinSet::Off => (0, r.mask),
                BinSet::Enable => (on_port, r.mask),
                BinSet::Fields(fields) => {
                    let m = fields.iter().fold(0u16, |acc, f| acc | (1u16 << f.bit));
                    (on_port, m)
                }
            };
            // A never-configured output can read back divisor 0; fall back to
            // 40 Hz so the first enable yields a valid, low-rate stream.
            let divisor = if r.divisor == 0 { 20 } else { r.divisor };
            if mode != 0 && divisor != r.divisor {
                println!("(binary rate was unset; defaulting to 40 Hz — use set-bin-hz to change)");
            }
            let cfg = format!("VNWRG,75,{mode},{divisor},01,{mask:04X}");
            let reply = transact_retry(
                &mut port,
                &build_command(&cfg),
                5,
                |l| l.starts_with("$VNWRG,75"),
                "device did not accept the binary config — a $VNERR 0x0C means it won't fit at \
                 this baud; lower or disable ASCII async (set-ascii=off) first, or raise --baud",
            )?;
            println!("RX: {reply}");
            report_bin_config(&Reg75 {
                async_mode: mode,
                divisor,
                groups: 1,
                mask,
            });
            if persist {
                save_to_flash(&mut port, &no_reply)?;
            }
        }

        Command::SetBinHz { hz, persist } => {
            let cur = transact_retry(
                &mut port,
                &build_command("VNRRG,75"),
                5,
                |l| parse_reg75(l).is_some(),
                &no_reply,
            )?;
            let r = parse_reg75(&cur).unwrap(); // OK: predicate ensured Some
            let divisor = (800 / hz) as u16; // OK: parser guaranteed hz divides 800
            let cfg = format!("VNWRG,75,{},{divisor},01,{:04X}", r.async_mode, r.mask);
            let reply = transact_retry(
                &mut port,
                &build_command(&cfg),
                5,
                |l| l.starts_with("$VNWRG,75"),
                "device did not accept the binary rate — a $VNERR 0x0C means the frame won't fit \
                 at this baud; raise --baud or trim fields (set-bin=…)",
            )?;
            println!("RX: {reply}");
            report_bin_config(&Reg75 {
                async_mode: r.async_mode,
                divisor,
                groups: 1,
                mask: r.mask,
            });
            if persist {
                save_to_flash(&mut port, &no_reply)?;
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
