# Chores-01

Chores-XX files use [Prose form](../../AGENTS.md#prose-form). They
contain discussions and notes on various chores in github compatible
markdown. There is also a [todo.md](../todo.md) file that tracks
tasks and in general there should be a chore section for each task
with the why and how this task will be completed.

## docs: RS-232 wedge root cause + RPi5 TTL port plan

Commits: [[3]]

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
- Next step: port `rw-vn100` to the RPi5 and re-run the baud
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
  C++ SDK — `rw-vn100` already replaces it by talking the
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

## feat: passive bench, composable command grammar

Commits: [[6]]

Today's `bench` is too complex: every run *mutates* the device
(configure → measure → restore), conflating measurement with
configuration, and each subcommand is its own process — one port
open per invocation, and each fresh-process reopen is a wedge
die-roll [[1]]. The redesign makes `bench` purely passive, breaks
configuration into composable `get-*`/`set-*` verbs, runs a whole
CLI line over a single connection, and adds file-backed named
states. We think running the line over one open cuts the
reconnect count that Issue #1 ties to wedges.

- **Passive bench.** `bench [SECS]` measures whatever is already
  streaming — no device writes, no restore — default 5 s. This
  removes the configure→measure→restore dance that is bench's
  current bulk and its only wedge surface.
- **Decomposed config verbs**, each owning one register concern,
  all written as `key=value` tokens (the eventual step-grammar
  spelling, adopted from the start):
    - `get-ascii` / `set-ascii=<MODE|off>` — reg 6 ASCII preset.
    - `get-ascii-hz` / `set-ascii-hz=<HZ>` — reg 7 ASCII async
      rate. Replaces the old `get-hz`/`set-hz`, which are dropped.
    - `get-bin` / `set-bin=<FIELDS|off>` — reg 75 binary field
      mask.
    - `set-bin-hz=<HZ>` — reg 75 rateDivisor (`800/HZ`); read it
      back via `get-bin`.
    - Bare `set-bin` enables binary with the *current* fields.
      Bare `set-ascii` is an error — reg 6 conflates on/off with
      the preset (0 = off), so there is no separate enable to flip;
      name a preset.
- **One connection per invocation** — the whole line opens the
  port once and runs its steps in order. See
  [Step grammar](#step-grammar-joined-tokens-space-separated-steps).
  Reading a register while that one connection is already streaming
  — and why rw-vn100 may discard stream data when it does — is in
  ARCHITECTURE.md [[9]].
- **Per-mode rate verbs.** Each output carries its rate in a
  different register, so the rate is an explicit per-mode verb —
  `set-ascii-hz` (reg 7) and `set-bin-hz` (reg 75 divisor) — and
  no verb is mode-dependent. See
  [Per-mode rate verbs](#per-mode-rate-verbs).
- **File-backed named states.** A TOML config holds named
  profiles. See [Named states](#named-states-file-backed--default).

### Step grammar: +-joined tokens, space-separated steps

The CLI line is one open connection and a sequence of steps. The
grammar rides on shell word-splitting so there is no custom
space handling.

- Each shell *word* is one **step**; steps execute left-to-right.
- Within a word, tokens joined by `+` apply together as one
  atomic unit; multiple `set-*` in the same word merge into a
  single device write.
- Comma stays reserved for value-internal lists
  (`set-bin=time,accel`); `+` is the only token joiner — never
  comma — so the grammar stays unambiguous and flat (the
  comma-nested alternative would make a value contain
  sub-assignments, i.e. recursion).
- Intra-step order is also left-to-right: `set-ascii-hz=50+bench`
  = apply, then measure.
- This unlocks a single-connection **sweep**:
  `save-state set-bin-hz=50+bench set-bin-hz=100+bench
  set-bin-hz=200+bench restore-state` — snapshot, three
  configure-and-measure steps, restore, all on one open.

### Per-mode rate verbs

Each output mode carries its rate in a different register, so the
rate is an explicit per-mode verb rather than one mode-dependent
`set-hz`.

- `set-ascii-hz=N` writes reg 7 (the ASCII async rate).
- `set-bin-hz=N` writes the rateDivisor (`800/N`) *inside* reg 75
  — a read-modify-write that preserves the field mask. Because it
  stands alone, the binary rate is settable without the `+`-join,
  so a device reaches e.g. 200 Hz binary through the verbs in the
  config-verbs step, not only once the step grammar lands.
- `set-bin` and `set-bin-hz` both touch reg 75. Run separately
  they are two writes (mask, then divisor); the `+`-join can merge
  `set-bin=…+set-bin-hz=N` into one write as an optimization, but
  correctness never depends on it.
- We considered a single **universal `set-hz`** (option A) mapping
  to reg 7 or the reg-75 divisor by mode, and dropped it: it made
  one verb mode-dependent and *required* the `+`-join to fold a
  binary rate into the reg-75 write, coupling the rate story to
  the step grammar. Per-mode verbs are each one register concern
  and stand alone.

### Named states (file-backed) + default

The `--config` file (default `./.rw-vn100-config.toml`, TOML,
hand-editable) holds a map of named profiles plus a `default`
key. Three verbs, three distinct jobs:

- `save-state[=NAME]` — capture the current *device* regs into
  NAME (or the default name).
- `set-state=NAME+set-…` — define NAME from *explicit* values,
  no device read.
- `restore-state[=NAME]` — apply NAME (or the default when no
  name) *to* the device.
- `default` means only "the profile bare `restore-state` uses" —
  **never** auto-applied. The tool stays passive by default;
  nothing writes to the device unless a `set-*`/`restore-state`
  verb asks. We think an auto-applied default would reintroduce
  the surprise-mutation path Issue #1 [[1]] warns about.
- The snapshot covers output config (reg 6 / 7 / 75). Baud
  (reg 5) is excluded from restore by default, because rewriting
  baud is itself the wedge trigger; `--baud` controls the link
  instead.

### Open questions (resolve before coding the relevant step)

- `set-bin` and `set-ascii` both present on one line: error
  ("pick one output to bench") vs measure both — leaning error.
- `set-bin` rejected with `$VNERR` 0x0C while ASCII async is still
  streaming: the device's combined-load fit check (sum of streams
  on the port — see
  [Bench combined-load fit check at low baud](#bench-combined-load-fit-check-at-low-baud))
  can veto a binary write that fits on its own.
  A standalone `set-bin` must *not* silently silence ASCII (that
  is the surprise mutation passivity protects against), so the
  lean is to surface the error with a hint ("ASCII async is using
  the budget — lower or disable it first") rather than auto-fix.
- Persist: apply volatile by default (reverts on power cycle,
  matches today's bench), with `--persist` available.
- `reset` / `factory-reset` reboot the device mid-line — constrain
  them to standalone / last.
- Passive binary frame-rate needs a frame length: reuse
  `get-bin`'s reg-75 parser to self-configure, or sniff the
  binary sync byte `0xFA`. v1 may report byte throughput +
  ASCII line rate and add decoded binary rate via the reg-75 read.


## refactor: split main.rs into lib modules

Commits: [[7]],[[8]],[[10]]

`main.rs` had grown to ~1890 lines, and the 0.3.0 bench redesign
adds a large new surface (the `get-*`/`set-*` verbs, the `+`-step
grammar, a TOML config, named states). We split the monolith into
a lib crate of focused modules first, so the new code lands in
clean modules instead of growing the single file. Each step is a
pure code-move — behavior unchanged, verified by the existing
tests + clippy + a smoke run.

- `proto.rs` — VN-100 protocol primitives: `checksum` /
  `vn_crc16`, command framing (`build_command`,
  `verify_checksum`), `error_description`, the register parsers,
  and the `Field`/`FIELDS` binary-output vocabulary.
- `transact.rs` — port I/O: `read_reply`, `transact`,
  `transact_retry`, `send_reboot_command`.
- `cli.rs` — `parse_args`, the `Command` enum, and help text.
- `bench.rs` — the `bench` command (passive, built in 0.3.0-5).
- `lib.rs` ties the modules together; `main.rs` is thin — parse
  args, open the port, dispatch.
- Ladder: proto (-1), transact (-2), cli + bench scaffold (-3);
  one module per commit so each diff is a reviewable move.


## feat: decompose output config into register verbs

Commits: [[11]]

The old one-shot `bench --bin …` was the only convenient way to
configure an output, coupling measurement to a
configure→measure→restore device mutation. This step breaks
configuration into composable verbs — one per register — so
configuration and measurement are separable, and the passive `bench`
to come (-5) has an easy way to turn an output on for testing. The
verb set and rationale are in the cycle design above
([Per-mode rate verbs](#per-mode-rate-verbs)).

- Six verbs, all `key=value` tokens (the eventual step-grammar
  spelling, adopted from the start): `get/set-ascii` (reg 6 preset),
  `set-ascii-hz` (reg 7 rate), `get/set-bin` (reg 75 field mask),
  `set-bin-hz` (reg 75 rateDivisor).
- `get-hz` / `set-hz` are dropped — `set-ascii-hz` is the same reg-7
  write under a mode-explicit name, and nothing depended on the old
  spelling.
- `set-bin` / `set-bin-hz` are read-modify-writes of reg 75
  (round-trip confirmed on the device at 921600):
    - `set-bin` preserves the divisor and sets the Common mask,
      enabling on port 2 (the RPi5 TTL header) when binary was off.
    - `set-bin-hz` preserves the port and mask, changing only the
      divisor.
- Passivity is preserved: a `set-bin` rejected by the device's
  combined-load fit check (`$VNERR` 0x0C) surfaces the error with a
  hint, rather than silently disabling ASCII async. See the parked
  [open question](#open-questions-resolve-before-coding-the-relevant-step).
- The legacy configure/measure `bench` stays as a parallel path
  until passive bench (-5) replaces it.
- On a parse error the CLI now prints just the error plus a `--help`
  pointer — the full help dump scrolled the error off a small screen.


## feat: passive bench measures the live stream

Commits:

`bench [SECS]` is now purely passive: it opens the port, reads the
live stream for `SECS` (default 5), and reports — no device writes,
no configure/measure/restore. It scans the same bytes two ways at
once — ASCII `$VN…` async lines (checksum-valid) and binary `0xFA`
Common-group frames (CRC-valid) — plus total wire throughput. This
removes bench's only wedge surface and lets either or both streams
be measured as they actually run.

- Binary is passive: the frame length is **sniffed** from each
  frame's own header (groups `0x01`, the 16-bit Common mask) and
  confirmed by CRC, rather than computed from a config. Resolves
  the `0xFA`-sniff open question [[1]].
- Drops the configure/measure/restore `bench`, its `--bin` /
  `--hz` / `--fields` / `--serial-port` / `--type` flags, the
  serial-port parser, and `default_fields`. The decomposed verbs
  (-4) are how you configure now.
- The `uncomp_accel` / `uncomp_gyro` sample fields gained their
  units (`m/s^2`, `rad/s`) — the original prompt for this work.
- Real device captures are committed under `test-data/` as test
  fixtures; a test parses `both-streams.bin` at every read-chunk
  size. See [test-data/README.md](/test-data/README.md).

### Intermittent zero-parse (open)

Passive `bench` intermittently reads bytes off the wire but parses
**zero** of them into ASCII messages or binary frames — *zero-parse*
— so it reports `ASCII: none / Binary: none`, then works on a
re-run. **The symptom varies, so there may be more than one cause**
— captured here so AM work starts from evidence, not a single
theory. Tracked as a Todo.

- **Full-throughput failures.** Twice (session start; a user run)
  it reported 0/0 at the *full* ~269 kbit/s — every byte read,
  none parsed.
- **Low-throughput failure.** Once it reported 0/0 at only
  ~24 kbit/s (≈2.4 KB/s) — at both `--baud 115200` and a following
  `--baud 921600` — and then a second identical `--baud 921600`
  re-run immediately returned to the full ~268 kbit/s and parsed
  fine. The low run came right after a *wrong-baud* (115200) open.
  We think a 921600 open immediately following a 115200 open does
  not always apply the new baud (PL011 divisor), so the 921600
  stream is read at the stale rate — low effective byte rate, all
  misframed — which the next open clears. This is a port-open /
  baud-sync issue, not the device changing state.

Ruled out for the **full-throughput** case:

- The parser — it counts the real captured streams correctly at
  every read-chunk size (`test-data/both-streams.bin`, test
  `measure_parses_real_both_streams_capture`).
- Byte loss / UART overrun — those failures show *full*
  throughput, and dmesg logs no overrun.
- Line-discipline corruption — the tty is in raw mode
  (`-icrnl -istrip …`).
- Idle backlog — a 60 s flooded-idle then bench parsed fine.
- A 115200→921600 transition — a wrong-baud open then a 921600
  bench parsed fine.

For the full-throughput case we *think* the port may come up
**bit-misaligned** on some cold opens (the baud not fully applied),
so the window is shifted garbage — full count, wrong values — which
re-opening clears; and that the old configure/measure `bench` was
shielded by its pre-measure `transact_retry` (which primed the port
and would fail loudly on a bad baud) while the passive bench reads
cold. This does **not** explain the low-throughput failure.
(Superseded — the confirmed subsection below shows framing stays
intact and the corruption is a B6 bit-flip, not a shifted window.)

Next steps (AM):

- **First, check the device** — `get-bin` / `get-ascii` / `rrg 7`
  / `rrg 5` (baud). The low-throughput failure suggests the device
  may have reset or drifted; rule that out before blaming the host.
- Add an env-gated raw dump (`RWVN100_DUMP=<file>`) to `measure`,
  capture a real failure, and diff it against the clean
  `both-streams.bin` — bit-shifted ⇒ a port-open / baud-sync fix
  (e.g. a verify-sync transact before measuring); clean ⇒ reopen
  the parser question.
- Replace the circular synthetic interleave tests with
  real-fixture tests, and split `measure` into a clock-free scanner
  so the fixture tests run instantly.

### Zero-parse root cause: PL011 baud-change open (confirmed 2026-06-28)

Both modes above reproduced on the rpi5/TTL using the `--capture`
flag (the AM `RWVN100_DUMP` idea, landed as `bench --capture
<path>`). The recipe makes every measured open a baud *change*:
alternate a `bench --baud 115200` (wrong baud — device at 921600)
immediately before each `bench --baud 921600`. 4 of 20 such opens
failed (20%); the streaming device never changed, and the next
open always parsed clean.

- **Full-misframe** (`test-data/zero-parse/misframe-fail.bin`) —
  "misframe" is a misnomer: framing is **intact**. All ~200 frames
  are present (tolerating a bit-6 flip on the header bytes recovers
  201 matches vs 130 for a strict `fa 01 39 07`), and CRLFs survive.
  The byte clock is right — instead, **bit 6 (B6)** flips
  intermittently. A byte's bits are numbered 0..7 LSB-first, so B6
  is the seventh. B6 is set on `0x2X/0x3X` data (`+`→`k`, digit→
  lowercase) and cleared on `0xFA`→`0xBA`, so XOR `0x40` recovers
  valid bytes, but over a ~110 B frame every CRC / checksum trips
  and 0 parse. The "full ~269 kbit/s, none parsed" case. Not a
  shifted/misaligned window (an earlier guess) — the divisor applied
  correctly.
- **Low / stale divisor** (`test-data/zero-parse/stale-fail.bin`) —
  ~2.2 KB read, no structure (1 `0xFA`, 3 `$` in the file): the
  921600 open kept the stale 115200 divisor and undersampled the
  stream 8×. The "~24 kbit/s after a wrong-baud open" case.

`test-data/zero-parse/` holds each mode as a clean→fail→clean
triplet (`*-before` / `*-fail` / `*-after`, every captured open
preceded by a 115200 flip) plus `repro.sh`, the script that
demonstrates it.

It is host-side, not the device: the stream is continuous, only
some opens fail, and a re-open clears it. The two modes differ,
though. The **low** case is a genuine divisor failure — the open
keeps the stale 115200 divisor, an 8× error that breaks framing.
The **misframe** case is not — framing is intact and the byte clock
is right, only B6 flips. We think both stem from a marginal
high-baud open (the low case a wholly-unapplied divisor, the
misframe case a sampling / signal-integrity glitch below the byte
layer), but the **B6 specificity is unexplained** — a baud/clock
error would smear across adjacent bits and break framing, which it
does not. This is why the old configure/measure bench was immune:
its pre-measure `transact_retry` was a write→read that forced a
clean lock (or failed loudly). The passive bench reads cold.

Re-confirmed from cold: with the VN-100 power-cycled to flash
defaults, `repro.sh` (default `START_BAUD=115200`) failed 2/20 (both
B6-flip), so the symptom is not an artifact of session state. A
config-phase `$VNERR 0x05` (not enough parameters) appeared on that
run too — we think the same corruption can also garble an *outgoing*
command (TX side), not just passive reads, though that is
unconfirmed.

Fix direction (the remaining Todo):

- **Reopen-on-bad-open** — `measure` detects 0 valid frames/lines
  (or throughput far below the link) in an early window and
  reopens the port, since a re-open clears it. Cheap, and matches
  the observed recovery.
- **Re-assert the baud after open** — force the termios divisor
  before reading (a second `set_baud_rate` / `tcsetattr`), so a cold
  open can't keep a stale divisor. This addresses the low case; the
  B6-flip case still needs reopen-on-bad-open.


## fix: binary output targets the wrong VN-100 serial port on TTL

Commits: [[4]]

First contact with the VN-100 from `rw-vn100` on the RPi5 flight
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
succeeded immediately. `rw-vn100` hardcodes `asyncMode=1`
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


## fix: bench silences async before binary config

Commits: [[5]]

`bench --bin` could reject a binary config with `$VNERR` 0x0C
("insufficient baud rate") at a low baud even when the binary stream
fits the link with room to spare. The cause was write ordering:
bench configured the binary output (reg 75) while ASCII async (reg
7) was still streaming, so the device's fit check saw the *combined*
ASCII + binary load. The fix silences ASCII async before writing the
binary config, so the check sees only the binary load.

### Bench combined-load fit check at low baud

Observed 2026-06-26 on the RPi5 TTL header (`/dev/ttyAMA0`) running
`bench --bin` after an in-session baud switch to 57600:

- At 57600 with ASCII async still at 40 Hz, `VNWRG,75,2,20,01,0101`
  (time+accel, 26 B/frame, 40 Hz) was rejected with `$VNERR` 0x0C.
- The binary stream alone is ~10 kbit/s — 18% of the 57600 link —
  so it fits easily; only the transient ASCII-40 + binary-40
  combination overran the check.
- Lowering ASCII async to 4 Hz first, then re-running the identical
  bench, the same `VNWRG,75` was accepted and streamed clean at
  39.9 Hz. Only the ASCII rate changed, isolating the combined load
  as the cause.

We think the device enforces `message_size × rate ≤ baud` on every
reg-5 / reg-7 / reg-75 write, against the *sum* of the streams
configured on that port. At 115200 the headroom hid the transient;
at 57600 it surfaced. The ordering was always latent and only became
visible at the lower baud.

The fix reorders `bench_binary`:

- Before: read reg 7 → write reg 75 → write reg 7=0. The fit check
  fired on the reg-75 write, before ASCII was silenced.
- After: read reg 7 → write reg 7=0 → write reg 75, and restore reg
  7 if the reg-75 write still fails — so a genuinely-too-big binary
  frame (e.g. the 38 B frame at 200 Hz / 115200, [[2]]) no longer
  leaves ASCII async switched off.
- `bench_ascii` is unaffected: it has a single stream and already
  writes the message type (reg 6) before the rate (reg 7).

## Rename rdwr_vn100 to rw-vn100

Commits: [[12]]

The renaming was because claude-code is converting the underscore to a
a hypen so we endup with two ~/.claude/projects and claude isn't using
our symlink to our /.claude repo.

So in our /.claude repo commit there are "new" .jsonl that were not
previously added.


## docs: fix ICD citations, add ARCHITECTURE.md

Commits: [[13]]

A verification pass against the VN-100 ICD and User Manual, with the
device confirmed on firmware 3.1.0.0. The register / enum / table
citations scattered through the code and REFERENCE.md were audited
against the authoritative sections, and the transaction-model design
was written up as its own document.

- ARCHITECTURE.md is new — the module map plus the transaction-model
  design: rw-vn100's transact-or-measure discard versus the vnsdk
  Listening Thread ([[9]]).
- Citations sharpened — framing to ICD §2.1.3 / §1.4.2-3, and the
  `vn_crc16`, `error_description`, `VALID_RATES`, `VALID_BAUDS` cites
  corrected to their real sections.
- Firmware provenance recorded — device 3.1.0.0 is the ICD baseline,
  with the User Manual's firmware-v1.1 caveat noted.

# References

[1]: /notes/bugs.md#issue-1--high-baud-reconnect-can-wedge-the-vn-100-uart
[2]: /notes/chores/chores-01.md#200-hz-binary-frame-budget-at-115200
[3]: https://github.com/winksaville/rw-vn100/commit/656853ed17a2 "656853ed17a2c4a6f36b5e4cf9c1ca0dbaf0d570"
[4]: https://github.com/winksaville/rw-vn100/commit/17d25b209c0c "17d25b209c0ca61aea3a8f84041bc7002226e78f"
[5]: https://github.com/winksaville/rw-vn100/commit/49e72e583d47 "49e72e583d4787fb567357965081983d3ee9e60b"
[6]: https://github.com/winksaville/rw-vn100/commit/ec6c523d4991 "ec6c523d499125093f5e9a3daac60e145dffaf40"
[7]: https://github.com/winksaville/rw-vn100/commit/cb3c720fefdf "cb3c720fefdf078c21475698c0675117588e988a"
[8]: https://github.com/winksaville/rw-vn100/commit/3e2c4983c744 "3e2c4983c744762a72dcfd4e3b670c6b0dc9e079"
[9]: /ARCHITECTURE.md#transaction-model-rw-vn100-discard-vs-the-vnsdk-listening-thread
[10]: https://github.com/winksaville/rw-vn100/commit/b8a24bbec5b8 "b8a24bbec5b8ce57fb622bfe8f2037a237b53f37"
[11]: https://github.com/winksaville/rw-vn100/commit/02b39b39b061 "02b39b39b061c3b5a304c9162cfb675a12b0cf89"
[12]: https://github.com/winksaville/rw-vn100/commit/c18ee71941b2 "c18ee71941b244a9f0e75ad5f328bc278ea3b721"
[13]: https://github.com/winksaville/rw-vn100/commit/7fd38e84f7b6 "7fd38e84f7b675dbc04ce4e213a37c46763601f5"

