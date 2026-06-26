# Chores-01

Chores-XX files use [Prose form](../../AGENTS.md#prose-form). They
contain discussions and notes on various chores in github compatible
markdown. There is also a [todo.md](../todo.md) file that tracks
tasks and in general there should be a chore section for each task
with the why and how this task will be completed.

## docs: RS-232 wedge root cause + RPi5 TTL port plan

Commits:

Benching the VN-100 baud climb wedged the device at 921600 —
silent at every baud until a power cycle. The investigation traced
the cause to the bench rig's RS-232 link, not the VN-100, and the
real flight target (RPi5) drives the IMU over a clean 3.3 V TTL
link — which reframes both the bug and the next step.

- Bug recorded as [[1]] (bugs.md Issue #1) with the full baud
  matrix and the recovery (power-cycle the VN-100 only; the FTDI
  adapter stays enumerated, which is what proved the wedge is in
  the device, not the host or adapter).
- The VN-100 firmware supports 921600 (Reg 5) and the in-session
  baud switch read cleanly at 921600 — so the device UART can do
  it. Only the cross-process *reopen* over RS-232 wedges. Cause:
  see [RS-232 link analysis](#rs-232-link-analysis).
- Next step: port `rdwr_vn100` to the RPi5 and re-run the baud
  climb on the TTL UART to test whether the wedge is RS-232-only.
  The tool is already portable (pure Rust + `serialport` crate);
  only the default `--port` (`/dev/ttyUSB0`, `main.rs:463`) is
  bench-specific. On the Pi the line is `/dev/ttyAMA0` (or the
  `/dev/serial0` symlink) — reachable today via `--port`. RPi5
  UART setup: free the line from the serial console
  (`enable_uart=1`, disable the console getty); 3.3 V logic only.
- On the Pi the IMU read path lives in existing Python (primary
  app `../fc/src/fc.py`, plus `../fc/scripts`). We do *not* need
  to port the whole app now, and we do *not* need the VectorNav
  C++ SDK — `rdwr_vn100` already replaces it by talking the
  VN-100 binary protocol directly in Rust. Near-term scope is
  just that: read the VN-100 from Rust over the Pi's TTL link.
  How it then feeds `fc.py` (IPC or a PyO3 module) is a later
  boundary question; a full Rust rewrite (the `fcbr` direction)
  is wanted eventually, not now.

### RS-232 link analysis

The bench adapter is a Gearmo GM-FTD12-LED-C — a USB-C to true
RS-232 adapter (±5.7 V output, rated 300 bps–460 Kbps on the spec
sheet; the "1 Mbps" is marketing-page only). The data lines are
RS-232 levels, so there is no TTL logic-level mismatch.

- Wiring: the VN-100 is on its RS-232 port (data crossed,
  adapter 2/3 ↔ VN-100 3/2), powered from a separate 5 V bench
  supply, common ground, no handshake lines.
- 921600 is ~2× the adapter's 460 Kbps rating. We think RS-232
  slew-rate limiting — on the adapter's transceiver *and* the
  VN-100's own onboard RS-232 transceiver, neither swappable —
  rounds the ±V waveform enough at 921600 (1.08 µs/bit) on the
  hand-made, unterminated cable to mis-frame bytes into the
  VN-100. Its firmware reacts to malformed framing by wedging.
- 57600 / 115200 / 230400 sit within the rating and reconnect
  cleanly; 921600 is 2/2 wedged. 230400 wedged once in a chained
  run with no power cycle between switches, so it is borderline.
- The RPi5 flight target avoids all of this: a direct 3.3 V TTL
  UART has no RS-232 transceivers in the path. We think the wedge
  will not reproduce on TTL; the port-to-Pi step is how we find
  out. Independent of that, binary output at 115200 already
  delivers 200 Hz, so high baud is not required for the flight
  goal — it is headroom, not a blocker.


## fix: binary output targets the wrong VN-100 serial port on TTL

Commits:

First contact with the VN-100 from `rdwr_vn100` on the RPi5 flight
target succeeded over the 3.3 V TTL header UART (`/dev/ttyAMA0`,
RP1 UART0 on GPIO14/15, `dtoverlay=uart0-pi5` loaded). ASCII paths
work bare; binary output needed a fix and surfaced a real frame
budget limit. Both are measured below.

- Read paths over TTL at 115200 are clean: `get-hz` returns
  40 Hz; the ASCII async `bench` measures 39.9 Hz on a full
  `$VNYMR` stream (~42% link use), no checksum/framing errors.
- `bench --bin` returned "0 frames" until the binary-output port
  was corrected — the tool hardcodes register-75 `asyncMode=1`,
  but the Pi's TX/RX link answers to port 2. See
  [register-75 serial-port numbering on TTL](#vn-100-register-75-serial-port-numbering-on-ttl).
- The full attitude+IMU set does not fit at 200 Hz / 115200: the
  device rejects it with a `$VNERR` 0x0C. See
  [200 Hz binary frame budget at 115200](#200-hz-binary-frame-budget-at-115200).
- The flight target needs only `/dev/ttyAMA0`; the tool's default
  `--port` is still the bench `/dev/ttyUSB0` (`main.rs:463`),
  passed explicitly on the Pi. Making the Pi port the default is
  an open ergonomics question, tracked in todo.

### VN-100 register-75 serial-port numbering on TTL

The VN-100 binary-output register (75) names a target serial port
in its first field (`asyncMode`): 1, 2, or 3 (both). ASCII async
(registers 6/7) instead targets whichever port the config command
arrived on, so it needs no port number and "just works" on the
connected wire — which is why `get-hz` and the ASCII `bench`
succeeded immediately. `rdwr_vn100` hardcodes `asyncMode=1`
(`main.rs:1060`, `bench_binary`), which on the Pi's wire emits no
binary at all.

Measured on `/dev/ttyAMA0`, identical field set (time+accel,
200 Hz), 2 s raw capture via `../fc/scripts/serial_check.py read`:

- `asyncMode=1` → 9760 B, ASCII `$VNYMR` only, no `0xFA` frames.
- `asyncMode=2` → 20145 B, `0xFA` binary frames present.
- `asyncMode=3` → 20151 B, `0xFA` binary frames present.

So the Pi's serial port (its one TX/RX pair on GPIO14/15) is the
VN-100's serial port **2** in the register-75 numbering; the bench
RS-232 rig used port 1.

Port 1 and port 2 are two *separate physical UART interfaces* on
the VN-100 — different pin sets — not two logical streams sharing
one line. We are wired to exactly one of them (port 2); port 1 is
the interface the bench RS-232 transceiver used, and the Pi is not
connected to it at all. What *is* interleaved on our one port is
two message *types*, not two ports:

- ASCII async (`$VNYMR`, registers 6/7) targets whichever port the
  config arrived on, so it lands on port 2 (us) and is always
  present on our line.
- Binary async (the `0xFA` frames, register 75) carries an
  explicit port-number field. With `asyncMode` 2 or 3 the binary
  is also directed to port 2, so our line then carries ASCII and
  binary time-multiplexed; with `asyncMode=1` the binary goes out
  port 1 (the interface we are not wired to), so we see ASCII only.

The byte counts confirm this is two types on one port, not two
ports: ASCII-only is ~4.9 kB/s (40 Hz YMR); the mode-2 / mode-3
captures add ~5.2 kB/s of binary (200 Hz × 26 B) on the *same*
line, doubling the total to ~10 kB/s. We think the VN-100 exposes
two UART interfaces with the RS-232 transceiver on UART1 and the
Pi's GPIO14/15 TTL lines on UART2 — inferred from the port-number
behavior, not from a pinout we have verified.

The fix is to stop hardcoding port 1: expose a
`--serial-port {1,2,both}` choice and pick a default that suits
the flight target (port 2, or `both` for robustness). The default
is a design decision for the fix cycle.

### 200 Hz binary frame budget at 115200

At 200 Hz the per-sample period is 5 ms; at 115200 baud 8N1
(11520 B/s) that is a raw budget of 57.6 bytes on the wire. The
VN-100 applies a more conservative internal margin than the raw
budget:

- time+accel (26 B/frame, 20 B payload) → accepted, streams clean
  at 199.9 Hz (~45% link use).
- accel+gyro (30 B/frame, 24 B payload) → accepted, streams clean
  at 199.9 Hz (~52% link use).
- time+accel+gyro (38 B/frame, 32 B payload) → rejected with
  `$VNERR` 0x0C ("insufficient baud rate").
- time+accel+gyro+quat (54 B/frame, 48 B payload) → rejected.

So the usable binary frame at 200 Hz / 115200 sits between 30 B
(accepted) and 38 B (rejected) — well under the 57.6 B raw budget,
i.e. the VN-100 keeps a large internal margin. In practice that is
time plus *one* 3-axis vector, or two vectors without time; a
third 12-byte vector busts it. Carrying time + accel + gyro (the
IMU essentials), let alone a quaternion, at 200 Hz needs a higher
baud — which reopens the high-baud reconnect wedge question
([[1]], Issue #1) on TTL this time, where we think it may not
reproduce.

The fit check is **per selected port**: `--serial-port 2` and
`--serial-port 1` reject the 38 B frame identically, so the 0x0C
is the frame budget, not the port. This is why the binary default
is `2` (the connected flight port), not `both`: `both` also
subjects port 1 to the check, so once port 2 is raised to a high
baud with port 1 left at 115200, `both` would let port 1 veto a
frame port 2 could take.

### As-built ladder

Landed as one commit (cycle 0.2.0):

- `feat: default RPi5 UART, fix binary port on TTL` — `--port`
  defaults to `/dev/ttyAMA0` (the RPi5 header UART); binary output
  gains `--serial-port {1,2,both}` (default `2`) replacing the
  hardcoded `asyncMode=1`; help text + tests updated. Verified
  bare on the Pi: `get-hz` 40 Hz, binary `accel,gyro` 199.9 Hz.


# References

[1]: /notes/bugs.md#issue-1--high-baud-reconnect-can-wedge-the-vn-100-uart

