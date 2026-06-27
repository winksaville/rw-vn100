# rw-vn100 architecture

How `rw-vn100` is put together and the design decisions that aren't
obvious from the code. Protocol values (registers, framing, error
codes) live in [REFERENCE.md](REFERENCE.md); this file is about the
tool's own structure and the *why* behind it.

## Module map

The crate is a thin binary over a library of focused modules
(`main.rs` → `lib.rs` → the rest):

- `main.rs` — thin entry point; parses args and calls into the lib.
- `lib.rs` — crate root and command dispatch (runs a parsed
  `Command` against an open port).
- `cli.rs` — arg parsing, the `Command` enum, and `--help` text.
- `proto.rs` — VN-100 protocol primitives: checksum/CRC, command
  framing, register parsers, and the `Field`/`FIELDS` table.
- `transact.rs` — port I/O: `read_reply`, `transact` /
  `transact_retry`, `send_reboot_command`.
- `bench.rs` — the `bench` command (measures the live stream).

## Transaction model: rw-vn100 discard vs the vnsdk Listening Thread

rw-vn100 and the VectorNav vnsdk sit at opposite points on one
question: can you read a register reply *while the device streams*?
It matters because the single-connection CLI line (e.g.
`set-bin=accel,gyro+set-hz=200 bench`) reads registers on a link
that may already be streaming.

- **rw-vn100 is single-threaded — transact *or* measure.**
  `read_reply` (`src/transact.rs`) accumulates bytes to a newline,
  slices the candidate from the last `$`, and drops every line
  that isn't the matched reply — so any binary frames or async
  ASCII arriving *during* a register transaction are discarded.
- **The vnsdk is threaded — transact *and* measure.** `connect`
  starts a Listening Thread (vnsdk `Interface/Sensor.hpp`) that
  reads every byte and dispatches by sync byte (Ascii / Fa `0xFA`
  / Fb `0xFB` PacketDispatchers) into a MeasurementQueue;
  `getMostRecentMeasurement` pops it. Commands ride a separate
  CommandProcessor, blocking by default. A blocking register read
  parks only the caller — the Listening Thread keeps filling the
  queue, so no measurement is lost. This is measured from the
  header, not inferred.
- **Why rw-vn100's discard is acceptable.** rw-vn100 is the
  pre-flight setup / diagnostic tool, never the in-flight data
  path. The flight code (`../fc/src/fc_2v3A0.py`) does all
  register I/O in `run()` setup (`get_model`,
  `set_reference_frame_rotation`, `set_initial_angle`) *before*
  `read_data()`; once that loop starts it only calls
  `getMostRecentMeasurement()` — no register I/O mid-stream. And
  even if it did, the SDK demuxes rather than discards. So
  rw-vn100 never needs simultaneous transact+measure.
- **Out of scope, on purpose.** A frame-aware demultiplexer for
  rw-vn100 (route `0xFA`/`0xFB` to a frame counter, `$VN…` lines
  to the command matcher) is therefore *not* planned; the passive
  bench's measurement loop and the register steps stay sequential
  on the one connection. rw-vn100 also ignores `0xFB` split
  packets (the SDK handles them) — acceptable for the same reason;
  see REFERENCE.md "Framing & checksums" for that caveat.
