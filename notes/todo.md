# Todo

This file uses [Prose form](../AGENTS.md#prose-form). It
contains near term tasks with a short description and
uses links or reference links for more details.

## In Progress

When a `## Todo` item is picked up, its text moves here: the
problem overview and its list of things to do. That is followed
by the "plan" — a bulleted list of the development "ladder":
   - 0.xx.y-0 blah (done)
   - 0.xx.y-1 blah blah (current)
   - 0.xx.y-2 blah blah blah
   - 0.xx.y close-out and validation

**feat: passive bench, composable command grammar**

Today's `bench` mutates the device on every run
(configure → measure → restore), conflating measurement with
configuration, and reopens the port per subcommand — each reopen
a wedge die-roll. Redesign `bench` to be purely passive,
decompose config into composable `get-*`/`set-*` verbs, run a
whole CLI line over a single connection, and add file-backed
named states. Full design + rationale in chores-01 [[1]].

A code reorganization comes first: `main.rs` (~1890 lines) is
split into a lib crate of focused modules, so the redesign's
large new surface (verbs, step grammar, TOML config, named
states) lands in clean modules instead of growing the monolith.
Design + module layout in chores-01 [[5]].

   - 0.3.0-0 prep: land the design note + this entry. (done)
   - 0.3.0-1 refactor: add `lib.rs` + thin `main.rs`; move the
     VN-100 protocol primitives (checksum/CRC, command framing,
     register parsers, `Field`/`FIELDS`) to `proto.rs`. (done)
   - 0.3.0-2 refactor: move port I/O (`read_reply`, `transact`,
     `transact_retry`, `send_reboot_command`) to `transact.rs`.
     (done)
   - 0.3.0-3 refactor: move arg parsing, the `Command` enum, and
     help text to `cli.rs`; scaffold `bench.rs`. (done)
   - 0.3.0-4 feat: decompose output config into register verbs —
     `get/set-ascii` (reg 6), `set-ascii-hz` (reg 7),
     `get/set-bin` (reg 75), and `set-bin-hz` (the reg-75
     divisor), all as `key=value` tokens. Drops `get-hz`/`set-hz`.
     Keeps the legacy configure/measure `bench` as a parallel path
     until -5. (done)
   - 0.3.0-5 passive bench: `bench [SECS]` measures the live
     stream only — drop the configure/measure/restore code;
     ASCII line-count + binary frame rate via a `0xFA` sniff
     (CRC-checked) + total wire throughput. Resolves the passive
     binary-rate open question [[1]] (0xFA sniff). Lands in
     `bench.rs`. Now exercisable via the -4 verbs (`set-bin`
     then `bench`). (done — impl landed; an intermittent
     cold-open 0/0 remains, tracked as Todo #1)
   - 0.3.0-6 step grammar + one connection: shell-word steps,
     `+` token join, single port open, left-to-right execution,
     merge `set-bin`+`set-bin-hz` in one word into a single reg-75
     write.
   - 0.3.0-7 named states: `--config` TOML profile map;
     `save-state` / `set-state` / `restore-state`; default =
     bare-`restore-state` target, never auto-applied; baud
     excluded from restore.
   - 0.3.0 close-out: README + `--help` rewrite, validation
     (cargo cycle), version bump.

## Todo

 Entries are in **strict priority rank** — #1 highest,
 descending. Reprioritize by moving an entry, then
 `vc-x1 fix-todo --no-dry-run notes/todo.md` to renumber.
 The numbers are positional rank, not stable IDs — to refer
 to a Todo, name it by its **title** (a greppable mention;
 a numbered list item has no anchor to link to), not its
 number. Long-tail entries
 live in [todo-backlog.md](todo-backlog.md). Use the
 [Prose Form in AGENTS.md](../AGENTS.md#prose-form); deeper
 detail goes in `notes/chores/chores-NN.md` design
 subsections (link via `[N]` ref).

1. Fix passive `bench` intermittent zero-parse: it sometimes
   reports `ASCII: none / Binary: none` then works on a re-run.
   The symptom varies — full ~269 kbit/s in some failures, a low
   ~24 kbit/s in another — so start by checking device state
   (`get-bin` / `get-ascii` / `rrg 5`), then capture a real
   failure via an env-gated raw dump and diff against the clean
   `test-data/both-streams.bin`. [[8]]

2. `set-bin-fields+=<FIELDS>` / `set-bin-fields-=<FIELDS>`: OR-in /
   mask-out Common fields incrementally instead of restating the
   whole set. The set-arithmetic generalizes to the other bitmask
   registers — Binary Output 2/3 (regs 76/77, identical layout)
   and reg 75's `asyncMode` serial-port mask — but not to
   `set-ascii` (reg 6 is a single-valued preset, not a bitmask).
   Builds on the orthogonal `set-bin` verbs. [[1]]

## Done

Completed tasks are moved from `## Todo` to here, `## Done`, as they are completed
and older `## Done` sections are moved to [done.md](done.md) to keep this file small.

- feat: default RPi5 UART, fix binary port on TTL [[2]],[[3]]
- fix: bench silences async before binary config [[4]]
- feat: decompose output config into register verbs [[6]]
- feat: passive bench measures the live stream [[7]]
- feat: split set-bin into fields / on-off verbs — `set-bin-fields`
  sets the mask, `set-bin=on`/`off` toggles streaming, keeping them
  orthogonal. ASCII stays as-is: reg 6 (ADOR) has no separate enable
  bit (0=off, N=preset), so it can't be orthogonal without stateful
  preset memory. [[1]]

# References

[1]: chores/chores-01.md#feat-passive-bench-composable-command-grammar
[2]: chores/chores-01.md#vn-100-register-75-serial-port-numbering-on-ttl
[3]: chores/chores-01.md#fix-binary-output-targets-the-wrong-vn-100-serial-port-on-ttl
[4]: chores/chores-01.md#fix-bench-silences-async-before-binary-config
[5]: chores/chores-01.md#refactor-split-mainrs-into-lib-modules
[6]: chores/chores-01.md#feat-decompose-output-config-into-register-verbs
[7]: chores/chores-01.md#feat-passive-bench-measures-the-live-stream
[8]: chores/chores-01.md#intermittent-zero-parse-open
