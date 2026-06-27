# rw-vn100

A small Rust CLI to read and configure a **VectorNav VN-100** IMU over a serial
port — and, just as importantly, a record of what we learned getting it to a
**reliable 200 Hz**.

```
cargo run -- get-hz
cargo run -- set-hz 40 --persist
cargo run -- bench --bin --hz 200
```

---

## TL;DR — the headline finding

The goal was 200 Hz of accelerometer data. The obvious path (crank the baud to
921600) turned out to be **the wrong one**. The right answer:

> **Stay at the rock-solid 115200 baud and switch the device from its fat default
> ASCII message to a compact _binary_ output. 200 Hz then fits in under half the
> link, where 200 Hz of the default ASCII message is ~2× over.**

`bench --bin` proves it on real hardware: **1000 frames in 5.00 s = 200.0 Hz**,
every frame CRC-valid, ~52 kbit/s of the ~115 kbit/s a 115200 line provides (~45%).

---

## Commands

| Command | What it does |
|---|---|
| `get-hz` | Read the async output rate (register 7). |
| `set-hz <HZ> [--persist]` | Write the async output rate (validated). `--persist` saves to flash. |
| `baud <NEW> [--persist]` | Change the device serial baud (register 5), switch this connection to it, and verify — without closing the port. |
| `rrg <ID>` | Generic: read any register, print its fields. |
| `wrg <ID> <P1> [P2 …]` | Generic: write any register. Sharp tool — e.g. `wrg 5 921600` bypasses the safe baud switch; use `baud`. |
| `bench [--bin] [--hz HZ] [--secs S] [--fields LIST]` | Configure an output and **measure** the achieved rate, then restore. ASCII async by default; `--bin` selects a binary output (register 75); `--fields` picks the binary field set. |
| `reset` | Reboot the sensor (`$VNRST`); reloads saved flash settings. |
| `factory-reset` | Restore **all** registers to factory defaults and reboot (`$VNRFS`). Reverts to 115200 + default output. Not undoable. |
| `help` / `--help` / `-h` | Usage. |

Binary `--fields` (Common group): `time, ypr, quat, gyro, accel, imu, magpres`.

Global options: `--port PORT` (default `/dev/ttyUSB0`), `--baud BAUD` (default
`115200` — this is the rate the **host** talks at; it must match the device's
*current* rate).

---

## VN-100 protocol primer

> For the **authoritative** register/enum/field values this tool relies on — with
> citations to the ICD (`../docs/`) and the vnsdk headers — see
> [`REFERENCE.md`](REFERENCE.md). For the tool's internal structure and the design
> decisions behind it (module map, transaction model), see
> [`ARCHITECTURE.md`](ARCHITECTURE.md).

**ASCII commands** look like `$<payload>*XX\r\n`, where `XX` is the 8-bit XOR
checksum of everything between `$` and `*`:

```
Read register 7:    $VNRRG,07*74          -> $VNRRG,07,40*5C
Write register 7:   $VNWRG,07,40*59       -> $VNWRG,07,40*59
Write register 5:   $VNWRG,05,921600*53   (serial baud)
Write binary out:   $VNWRG,75,1,4,01,0101 (register 75, see below)
Save to flash:      $VNWNV*57             (writes ALL current registers)
Reboot:             $VNRST*4D
Factory reset:      $VNRFS*5F
Error reply:        $VNERR,<code>*XX
```

**Binary output** (configured via register 75) is a packed frame:

```
0xFA | groups | field-mask(s) | payload… | CRC16
```

- `0xFA` = sync byte.
- `groups` = bitmask of which field groups follow (`0x01` = the "Common" group).
- one 16-bit `field-mask` per group, little-endian.
- payload = the selected fields, in bit order, little-endian (`u64` time,
  `f32` floats).
- CRC16 = VectorNav's CRC-CCITT/XMODEM. A valid frame, run from the `groups`
  byte through the trailing CRC, produces **0**.

Our `bench` frame is Common group with **TimeStartup (`u64`, 8 B) + Accel
(`3×f32`, 12 B)** → `1+1+2+8+12+2 = 26 bytes`.

---

## What we learned (the useful part)

### 1. "Rate" is overloaded — there are two of them
- **Async output rate** (register 7): how often the device emits a message (Hz).
- **Serial baud rate** (register 5): how fast bytes move on the wire.

`set-hz 40` changes the first; `baud 921600` changes the second; `--baud 921600`
is just *the host connection speed* and changes **nothing** on the device.

### 2. The VN-100 ships at 40 Hz, and "200 Hz" isn't a frequency limit — it's bandwidth
The factory default async rate is **40 Hz**. Trying `set-hz 100` or `set-hz 200`
at 115200 returns `$VNERR,0C` = **"insufficient baud rate."** That's not "100 Hz
is too fast" — it's "100 Hz × *this message's bytes* exceeds the link."

At 115200, 8N1 (~10 bits/byte) → ~**11,520 bytes/s** usable:

| Message | Size | @ 200 Hz | Fits? |
|---|---|---|---|
| `VNYMR` (default ASCII) | ~115 B | ~23,000 B/s (~230 kbit/s) | ❌ ~2× over |
| Compact binary (time+accel) | 26 B | 5,200 B/s (~52 kbit/s) | ✅ ~45% of link |

This also explains the ladder we saw: `set-hz 40` ✅, `set-hz 50` ✅, `set-hz 100`
❌ (right at the wall), `set-hz 200` ❌.

### 3. The fix: send *less per sample*, not push more baud
ASCII presets that include acceleration are all big. **Binary output lets you
pick exactly the fields you need** (e.g. timestamp + accel), so 200 Hz fits at
115200 with ~55% headroom. The win isn't fewer total bytes — 200 Hz of the 26-B
binary frame (~52 kbit/s) is actually a touch *more* than 40 Hz of the ~115-B
VNYMR (~46 kbit/s) — it's **5× the sample rate for comparable bandwidth**.

### 4. The 921600 detour was a dead end on this hardware — but *only* at 921600
The USB adapter is an **FTDI FT232R**, which supports 921600 fine. Yet:
- A **volatile** baud change (no `--persist`) that switched in-process *verified*
  at 921600 — but once the port closed and a fresh process reopened, the device's
  UART ended up **wedged and silent at every baud**, needing a **power cycle**.
  (Independently reproduced with pyserial, so it wasn't this tool's bug.)
- **This is specific to the high baud, not to "volatile" itself.** A volatile
  change to **57600** holds perfectly across *repeated* fresh-process reconnects —
  the device keeps its RAM baud across host port closes; it only reverts on a
  power cycle or a `reset`/`factory-reset`. So 921600 isn't *reverting* on
  reconnect, it's the reconnect transient (DTR toggle / glitch) **corrupting the
  link** at a baud where timing margin is ~16× tighter.
- Lesson: lower/standard bauds reconnect reliably; treat ~921600 on this
  cable/adapter as fragile. If you ever truly need it, **persist it** so the
  device *boots* there (no reconnect-at-speed), or do it inside one managed
  session (the SDK's `changeBaudRate`). For our 200 Hz goal, none of this matters —
  binary-at-115200 wins on speed *and* robustness.

**Summary — 921600 is intermittent, and idle time matters.** It is *not* a clean
"always fails": 921600 sometimes reconnects fine on back-to-back runs (e.g.
`sleep 1` between them). But a **long idle gap between runs reliably fails** —
with `sleep 10` it wedged every time (3/3 attempts), while the immediately-prior
`get` in the same sequence succeeded. So the high-baud failure is *probabilistic*
and appears *time-dependent* (quick reconnects can survive; long-idle ones don't),
and once it wedges, only a power cycle recovers it. An intermittent,
unrecoverable failure that passes quick testing is the worst kind for flight, so
the conclusion stands: **stay at 115200** (zero observed glitches; binary already
gives 200 Hz there).

Here's an example: three consecutive `rw-vn100` runs — 115200 (baseline), then
921600 with a 1 s idle between gets, then 921600 with a 10 s idle. What changes
between the two 921600 runs is the **idle, not the baud**. Each run starts from
115200 (a power cycle reverts the volatile baud to the flash default).


Here we use 115200:
```
wink@3900x 26-06-21T16:47:23.024Z:~/data/prgs/nps-gnc/rw-vn100 (main+1)
$ rw-vn100 baud 115200; rw-vn100 --baud 115200 get; sleep 3; rw-vn100 --baud 115200 get;
Opening /dev/ttyUSB0 at 115200 baud...
TX: $VNWRG,05,115200*58
RX: $VNWRG,05,115200*58
Device acknowledged baud change to 115200.
Verifying at 115200 baud...
TX: $VNRRG,07*74
RX: $VNRRG,07,40*5C
Verified — device is at 115200 baud (async rate 40 Hz).
(Volatile — a power cycle or port reset reverts to flash. Re-run with `baud 115200 --persist` to make it permanent.)
Opening /dev/ttyUSB0 at 115200 baud...
TX: $VNRRG,07*74
RX: $VNRRG,07,40*5C
Async output rate: 40 Hz
Opening /dev/ttyUSB0 at 115200 baud...
TX: $VNRRG,07*74
RX: $VNRRG,07,40*5C
Async output rate: 40 Hz
wink@3900x 26-06-21T16:48:12.480Z:~/data/prgs/nps-gnc/rw-vn100 (main+1)
```

Here is 921600 with a sleep 1 between the 2nd and 3rd runs, works fine:
```
wink@3900x 26-06-21T16:48:12.480Z:~/data/prgs/nps-gnc/rw-vn100 (main+1)
$ rw-vn100 --baud 115200 baud 921600; rw-vn100 --baud 921600 get; sleep 1; rw-vn100 --baud 921600 get;
Opening /dev/ttyUSB0 at 115200 baud...
TX: $VNWRG,05,921600*53
RX: $VNWRG,05,921600*53
Device acknowledged baud change to 921600.
Verifying at 921600 baud...
TX: $VNRRG,07*74
RX: $VNRRG,07,40*5C
Verified — device is at 921600 baud (async rate 40 Hz).
(Volatile — a power cycle or port reset reverts to flash. Re-run with `baud 921600 --persist` to make it permanent.)
Opening /dev/ttyUSB0 at 921600 baud...
TX: $VNRRG,07*74
RX: $VNRRG,07,40*5C
Async output rate: 40 Hz
Opening /dev/ttyUSB0 at 921600 baud...
TX: $VNRRG,07*74
RX: $VNRRG,07,40*5C
Async output rate: 40 Hz
wink@3900x 26-06-21T16:48:39.584Z:~/data/prgs/nps-gnc/rw-vn100 (main+1)
```

Power-cycled (back to 115200), then changed to 921600 again — but now with a
`sleep 10` between the gets. This fails: tried it 3 times, never worked.
```
wink@3900x 26-06-21T16:48:39.584Z:~/data/prgs/nps-gnc/rw-vn100 (main+1)
$ rw-vn100 --baud 115200 baud 921600; rw-vn100 --baud 921600 get; sleep 10; rw-vn100 --baud 921600 get;
Opening /dev/ttyUSB0 at 115200 baud...
TX: $VNWRG,05,921600*53
RX: $VNWRG,05,921600*53
Device acknowledged baud change to 921600.
Verifying at 921600 baud...
TX: $VNRRG,07*74
RX: $VNRRG,07,40*5C
Verified — device is at 921600 baud (async rate 40 Hz).
(Volatile — a power cycle or port reset reverts to flash. Re-run with `baud 921600 --persist` to make it permanent.)
Opening /dev/ttyUSB0 at 921600 baud...
TX: $VNRRG,07*74
RX: $VNRRG,07,40*5C
Async output rate: 40 Hz
Opening /dev/ttyUSB0 at 921600 baud...
TX: $VNRRG,07*74
  attempt 1/5: no response yet, retrying...
TX: $VNRRG,07*74
  attempt 2/5: no response yet, retrying...
TX: $VNRRG,07*74
  attempt 3/5: no response yet, retrying...
TX: $VNRRG,07*74
  attempt 4/5: no response yet, retrying...
TX: $VNRRG,07*74
Error: "no usable reply from device — is it actually at 921600 baud? (VN-100 factory default is 115200; use --baud to match, or the `baud` command to change it) (after 5 attempts; last: no reply yet)"
wink@3900x 26-06-21T16:53:45.802Z:~/data/prgs/nps-gnc/rw-vn100 (main+1)
```

### 5. Talking to a streaming device needs a robust reader
Real serial I/O bites you in small ways we hit and fixed:
- A read returning `Ok(0)` or `TimedOut` is **not EOF** — keep waiting until an
  overall deadline. (Treating `Ok(0)` as EOF dropped slightly-late replies.)
- A fresh open (especially at high baud) can lose the **first** query while the
  USB chip settles — so commands **retry** (with a short settle + input flush).
- Frames split across USB reads — accumulate into a buffer and resync on a bad
  CRC (the sync byte can appear inside payload data).

### 6. VNERR codes are worth decoding
The tool maps `$VNERR,<hex>` to text (e.g. `0x0C` → "insufficient baud rate")
with a hint, instead of leaving you to look it up.

### 7. Why `../fc/src/fc.py` only ever saw 40 Hz
Its `VecNavHandler` uses the SDK's `autoConnect()` (which probes baud rates and
finds the device at its default 115200) and **never writes the output rate** to
the device — the `rate=200` / `baudrate=921600` constructor args are effectively
no-ops. So the device just streams its 40 Hz default and the host paces reads.
To get 200 Hz, *something* has to configure the device (register 7 for ASCII, or
register 75 for binary) — which is exactly what `bench` does.

---

## The `bench` proof, annotated

```text
$ cargo run -- bench --bin --hz 200 --secs 5
TX: $VNWRG,75,1,4,01,0101*70             # binary: Common[time,accel] @ 800/4 = 200 Hz
Configured binary output: Common["time", "accel"] @ 200 Hz (divisor 4, 26 B/frame).
TX: $VNWRG,07,0*6D                       # silence the ASCII stream while measuring
Measuring for 5s...

Result: 1000 frames in 5.00s = 200.0 Hz (target 200 Hz).
Sample: t=1718200006000 ns, accel=[9.264, -0.571, 1.095] m/s^2
Wire throughput ~52 kbit/s = 45% of the 115.2 kbit/s 115200-baud link.
Restored: binary output off, ASCII async back to 40 Hz.
```

Decoding that sample frame byte-for-byte:

| Offset | Bytes | Field | Value |
|---|---|---|---|
| 0 | `FA` | sync | — |
| 1 | `01` | groups | Common group present |
| 2–3 | `01 01` | field mask `0x0101` LE | TimeStartup(bit0)+Accel(bit8) |
| 4–11 | `70 75 B3 0C 90 01 00 00` | `u64` LE | 1,718,200,006,000 ns ≈ **1718 s uptime** |
| 12–15 | `58 39 14 41` | `f32` LE | Accel X = **9.264** m/s² |
| 16–19 | … | `f32` LE | Accel Y = **−0.571** m/s² |
| 20–23 | … | `f32` LE | Accel Z = **1.095** m/s² |
| 24–25 | CRC16 | — | whole-frame CRC = 0 |

`|accel|` ≈ 9.35 m/s² ≈ g — a stationary IMU measuring gravity, mostly along +X.
The Y value matches `fc.py`'s `Accel Y ≈ -0.6`, confirming the same channel.

> **Note on the IMU base rate:** binary `rateDivisor` divides the VN-100's
> internal **800 Hz** sample rate. `rateDivisor = 4` → 200 Hz. With `--bin`,
> `--hz` must divide 800 (e.g. 100, 200, 400); for the ASCII async output it must
> be one of the fixed register-7 values (`1,2,4,5,10,20,25,40,50,100,200`).
>
> **Pick the data with `--fields`** (binary only): e.g.
> `bench --bin --hz 200 --fields time,accel,gyro,quat`. The device accepts or
> rejects (`$VNERR,0C`) per its own bandwidth check, so you can map exactly what
> fits a given baud. A richer frame may need a higher baud or a lower rate.

---

## Recovering a confused device

- **`reset`** — reboot, keep saved settings.
- **`factory-reset`** — wipe to factory defaults (back to 115200 + default
  output). Issue it at the device's *current* baud.
- **Power cycle** — reloads flash; clears a wedged UART. (Note: a power cycle does
  **not** restore factory defaults — it reloads whatever is in flash.)

Nothing in this session was persisted, so a power cycle always returned the
device to a clean 115200 / 40 Hz.

---

## Build & test

```
cargo build
cargo test        # 22 unit tests, no hardware required
cargo run -- help
```

Dependency: [`serialport`](https://crates.io/crates/serialport).

Source is a single `src/main.rs`. Notable pieces: `checksum`/`verify_checksum`
(ASCII), `vn_crc16` (binary), `transact`/`transact_retry` (robust request/reply),
`read_reply` (deadline-bounded line reader), `measure_binary` + `run_bench`
(the rate proof).

---

## Where this is headed

We are **not** planning to modify `fc.py`. The likely next step is a separate
project — **`fcbr`** ("flight controller, binary, Rust") — that reads the VN-100
via this **binary** path in Rust, reusing the frame/CRC handling proven here,
instead of the Python SDK. Before building on the 200 Hz result, it's still worth
cross-checking it with an independent reader (the vendor SDK and/or a from-scratch
pyserial parser).
