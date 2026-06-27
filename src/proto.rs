//! VN-100 wire-protocol primitives.
//!
//! - ASCII command framing: `checksum`, `build_command`, `verify_checksum`.
//! - Replies / errors: `parse_reg06`, `parse_reg07`, `error_description`,
//!   `vnerr_message`.
//! - Binary output: `vn_crc16` and the `Field` / `FIELDS` field vocabulary.

/// A selectable binary-output field (all from the "Common" group, group 1):
/// CLI name, the group-1 bit it occupies, and its on-wire byte size.
pub struct Field {
    pub name: &'static str,
    pub bit: u8,
    pub size: usize,
}

/// The `--fields` vocabulary (Common group only — keeps the frame to one group).
/// Authoritative: REFERENCE.md "Common Group" (ICD §2.2 Table 2.3).
pub const FIELDS: &[Field] = &[
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

pub fn lookup_field(name: &str) -> Option<&'static Field> {
    FIELDS.iter().find(|f| f.name == name)
}

pub fn field_names() -> String {
    FIELDS.iter().map(|f| f.name).collect::<Vec<_>>().join(", ")
}

/// Parse a comma-separated `--fields` list into Common-group fields, ordered by
/// bit (the order the device emits them), de-duplicated.
pub fn parse_fields(list: &str) -> Result<Vec<&'static Field>, String> {
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
pub fn default_fields() -> Vec<&'static Field> {
    vec![
        lookup_field("time").unwrap(),
        lookup_field("accel").unwrap(),
    ]
}

/// Compute the VN-100 checksum: XOR of all bytes in `payload`.
pub fn checksum(payload: &str) -> u8 {
    payload.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// Build a full command line (including `$`, `*`, checksum and CRLF) from the
/// payload that sits between `$` and `*`, e.g. `"VNRRG,07"`.
pub fn build_command(payload: &str) -> String {
    format!("${}*{:02X}\r\n", payload, checksum(payload))
}

/// Verify the trailing `*XX` checksum of a received `$...*XX` line.
pub fn verify_checksum(line: &str) -> Result<(), String> {
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
/// Authoritative: REFERENCE.md "Error responses" (ICD §1.5 Table 1.6; vnsdk Errors.hpp Error).
pub fn error_description(code: u8) -> &'static str {
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
pub fn vnerr_message(line: &str) -> String {
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
pub fn parse_reg07(line: &str) -> Option<u32> {
    let body = line
        .strip_prefix("$VNRRG,07,")
        .or_else(|| line.strip_prefix("$VNWRG,07,"))?;
    let freq = body.split('*').next()?;
    freq.trim().parse().ok()
}

/// Parse register 6 (Async Data Output Type / ADOR) value from a reply.
pub fn parse_reg06(line: &str) -> Option<u8> {
    let body = line
        .strip_prefix("$VNRRG,06,")
        .or_else(|| line.strip_prefix("$VNWRG,06,"))?;
    body.split('*').next()?.trim().parse().ok()
}

/// VectorNav 16-bit CRC (CRC-CCITT/XMODEM, the algorithm from their app note).
/// A valid binary packet, run from the groups byte through the trailing CRC,
/// produces 0. Authoritative: REFERENCE.md "Framing & checksums"
/// (ICD §2.1.3 message format; §1.4.3 CRC16).
pub fn vn_crc16(data: &[u8]) -> u16 {
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
    fn fields_parse_orders_by_bit_and_dedups() {
        let f = parse_fields("accel,time,accel").unwrap();
        let names: Vec<&str> = f.iter().map(|x| x.name).collect();
        assert_eq!(names, vec!["time", "accel"]); // bit-ordered + de-duplicated
        let mask: u16 = f.iter().fold(0, |m, x| m | (1u16 << x.bit));
        assert_eq!(mask, 0x0101); // time bit0 + accel bit8 — matches the known config
        assert!(parse_fields("bogus").is_err());
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
}
