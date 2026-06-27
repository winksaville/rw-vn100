# VN-100 protocol reference (tool-scoped)

The authoritative values `rw-vn100` relies on, with citations. **Scope:** only
what this tool touches — not the whole VN-100 protocol. For anything else, go to
the primary sources directly.

**Verified 2026-06-27 against:**
- **ICD** — `../docs/VN100-ICD-v3_1_0_0-ICD10005-R1.pdf` (v3.1.0.0, R1, 2024-02-27).
  The authoritative protocol/register spec. Cited by **section number**.
- **vnsdk** — `../vnsdk/cpp/include/vectornav/Interface/{Registers,Errors}.hpp`
  (vnsdk **v1.2.0**, see `../vnsdk/changelog.txt`). Cited by **symbol name**
  (class/enum), never line number — line numbers rot across SDK releases.
- **User Manual** — `../docs/VN100-T_UserManual-UM001.pdf` (UM001, Rev 1.2.8,
  **Firmware v1.1**). Higher-level guide for **older firmware than our device**
  (see "Firmware version & document provenance" below) — used only where the ICD
  is silent, and re-verified against the ICD or the live device.

> Citation form: `(ICD §X; vnsdk Class::Enum)`. When re-verifying against a newer
> ICD/SDK, update the "Verified" date above and any values that changed.

A ✓ means **empirically confirmed** on real hardware (a `bench` run decoded
correctly); otherwise the value is doc-confirmed from the sources above.

---

## Firmware version & document provenance

The VN-100 ships protocol docs **per firmware version**, and the two we have
disagree: the ICD is for Firmware **v3.1.0.0**, the User Manual for Firmware
**v1.1**. The tie-breaker is the device itself — read live from its
identification registers (`rw-vn100 rrg <id>`):

| reg | field | ICD § | our device |
|---|---|---|---|
| 1 | Model Number | §4.2.1 | `VN-100S-CR` ✓ |
| 2 | Hardware Version | §4.2.2 | `7` ✓ |
| 3 | Serial Number | §4.2.3 | `0100130608` ✓ |
| 4 | Firmware Version | §4.2.4 | `3.1.0.0` ✓ |

- **Firmware 3.1.0.0 matches the ICD**, so the ICD is the authoritative match
  for this device — trust it first. Every register/enum value in this file is
  ICD-sourced and (where marked ✓) confirmed on this firmware.
- **The User Manual (UM001, Rev 1.2.8) targets Firmware v1.1** — two major
  versions behind. We think UM-sourced facts may not hold on firmware 3.1.0.0,
  so the UM is used only for what the ICD lacks, then re-verified.
- Re-check the device version any time with `rw-vn100 rrg 4` (UM §7.5 / ICD
  §4.2.4 document the same register). If a future device reports a different
  firmware, re-verify this whole file against the matching ICD.

---

## Framing & checksums — ICD §2.1.3 (binary message format), §1.4.2/§1.4.3 (checksums)

The binary **message format** (sync byte, group/type header, payload) is ICD
§2.1.3 — *not* §1.4, which covers only checksums. The **User Manual (UM001)
omits the binary framing entirely** and never points to the ICD, so the sync
byte is not findable there; go to the ICD.

- **ASCII command:** `$<payload>*XX\r\n`. `XX` is an **8-bit XOR** over every byte
  *between* `$` and `*` (commas included; ICD §1.4.2). Default on the UART.
  (Register 30 can switch checksum mode; this tool assumes the default.) ✓
- **Binary message** (ICD §2.1.3): `0xFA | groups | <16-bit type word per group> | payload | CRC16`.
  The `0xFA` **sync byte** is the first byte; a *split* packet uses sync `0xFB`
  instead (ICD §2.1.3, "split packet") — our passive sniff keys on `0xFA` only.
  The trailing **16-bit CRC** is used regardless of Register 30 (ICD §1.4.3,
  CRC16-CCITT). The CRC covers everything **after** the sync byte; running it
  over `groups…payload…CRC` yields **0** for a valid frame. ✓
- Code: `checksum()` / `verify_checksum()` (ASCII), `vn_crc16()` (binary).

## Register 5 — Serial Baud Rate — ICD §3.2.2 (Table 3.3); vnsdk `BaudRate::BaudRates`
`9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600`
Code: `VALID_BAUDS`. (Factory default 115200.)

## Register 6 — Async Data Output Type / ADOR — ICD §3.2.3 (Table 3.6); vnsdk `AsyncOutputType::Ador`
A **single** selection (one preset *or* off), not a bitmask. **Default = 14 (YMR).**

| ADOR | value | message (source register) |
|---|---|---|
| OFF | 0 | async off |
| YPR | 1 | Yaw,Pitch,Roll (reg 8) |
| QTN | 2 | Quaternion (reg 9) |
| QMR | 8 | Quat,Mag,Accel,Rates (reg 15) |
| MAG | 10 | Magnetic (reg 17) |
| ACC | 11 | Acceleration (reg 18) |
| GYR | 12 | Angular Rate (reg 19) |
| MAR | 13 | Mag,Accel,Rates (reg 20) |
| YMR | 14 | YPR,Mag,Accel,Rates (reg 27) — **default** |
| YBA | 16 | YPR,Body Accel,Rates (reg 239) |
| YIA | 17 | YPR,Inertial Accel,Rates (reg 240) |
| IMU | 19 | IMU Measurements (reg 54) |
| DTV | 30 | Delta Theta & Delta Velocity (reg 80) |
| HVE | 34 | Heave (reg 115) |

Code: `ASCII_TYPES` (`bench --type`). The SDK also defines GPS/INS values
(GPS/GPE/INS/INE/ISL/ISE/G2S/G2E) — **not applicable to the VN-100** (no GNSS).

## Register 7 — Async Data Output Frequency / ADOF — ICD §3.2.4 (Table 3.9); vnsdk `AsyncOutputFreq::Adof`
`0(off), 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200` Hz. **Max 200.** ✓ (40 default)
Code: `VALID_RATES` (non-zero values; `set-hz` / ASCII `bench`).

## Registers 75/76/77 — Binary Output 1/2/3 — ICD §3.2.8–3.2.10 (registers), §2 (message format); vnsdk `BinaryOutput1/2/3`
Write fields: `asyncMode` (serial-port bitmask), `rateDivisor`, then a field mask
per selected group. **Output rate = 800 / rateDivisor** (800 Hz IMU base; so
`rateDivisor 4` → 200 Hz). ✓ Three independent outputs, each its own rate.

### Common Group (group byte `0x01`) — ICD §2.2 (Table 2.3)
Bit offsets within the Common field mask, and on-wire sizes:

| bit | field | content | bytes | tool name |
|---|---|---|---|---|
| 0 | TimeStartup | `u64` ns | 8 | `time` ✓ |
| 2 | TimeSyncIn | `u64` ns | 8 | — |
| 3 | Ypr | 3×`f32` deg | 12 | `ypr` ✓ |
| 4 | Quaternion | 4×`f32` | 16 | `quat` ✓ |
| 5 | AngularRate | 3×`f32` rad/s | 12 | `gyro` ✓ |
| 8 | Accel | 3×`f32` m/s² | 12 | `accel` ✓ |
| 9 | Imu | UncompAccel(12)+UncompGyro(12) | 24 | `imu` |
| 10 | MagPres | Mag(12)+Temp(4)+Pres(4) | 20 | `magpres` |
| 11 | Deltas | DeltaTheta+DeltaVel | 28 | — |
| 13 | SyncInCnt | `u32` | 4 | — |

Sub-field sizes per ICD §2.4 (UncompAccel/UncompGyro = 12 each; Pressure = 4).
Frame length = `1 (sync) + 1 (groups) + 2 (field mask) + payload + 2 (CRC)`.
Code: `FIELDS` (`bench --bin --fields`).

## Error responses — ICD §1.5 (Table 1.6, $VNERR); vnsdk `Errors.hpp` `Error`
`$VNERR,<code>*XX` — `<code>` is **hex**.

| code | meaning | | code | meaning |
|---|---|---|---|---|
| 0x01 | hard fault | | 0x08 | invalid register |
| 0x02 | serial buffer overflow | | 0x09 | unauthorized access |
| 0x03 | invalid checksum | | 0x0A | watchdog reset |
| 0x04 | invalid command | | 0x0B | output buffer overflow |
| 0x05 | not enough parameters | | **0x0C** | **insufficient baud rate** |
| 0x06 | too many parameters | | 0xFF | error buffer overflow |
| 0x07 | invalid parameter | | | |

`0x0C` is the common one here: too much data for the current baud. Code:
`error_description()` / `vnerr_message()`.

## Commands (no register) — ICD §1.3
- `$VNWNV` — Write Settings (save all registers to non-volatile flash). §1.3.3
- `$VNRST` — Reset (reboot; reloads flash). §1.3.5
- `$VNRFS` — Restore Factory Settings (defaults + reboot). §1.3.4
- `$VNRRG,<id>` / `$VNWRG,<id>,…` — generic Read/Write Register (`rrg` / `wrg`). §1.3.1 / §1.3.2
