//! Serial transaction helpers — send a command and read its reply.
//!
//! - `read_reply` does deadline-bounded raw reads so unmatched garbage
//!   can't block forever.
//! - `transact` / `transact_retry` send a command and validate the reply
//!   (checksum, `$VNERR`); `send_reboot_command` tolerates a missing echo.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use crate::proto::{verify_checksum, vnerr_message};

/// Read from the device until `matches` accepts a line or the deadline passes.
///
/// Reads raw bytes (not `read_line`) so a stream of garbage with no newline —
/// e.g. when the host baud doesn't match the device — can't block us forever:
/// we honor an overall `deadline` and cap line length to drop runaway junk.
pub fn read_reply<R, F>(
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
                continue;
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
pub fn transact<S: Read + Write>(
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
pub fn transact_retry<S: Read + Write>(
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
pub fn send_reboot_command<S: Read + Write>(
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

#[cfg(test)]
mod tests {
    use super::*;

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
