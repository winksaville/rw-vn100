# test-data

Raw serial captures from a VN-100 (firmware 3.1.0.0) on the RPi5 TTL
header (`/dev/ttyAMA0`) at 921600 baud, ~1 s each, captured with
`stty -F /dev/ttyAMA0 921600 raw -echo` then `cat`. Used as real-byte
fixtures for `bench` parsing tests (via `include_bytes!`), so the
scanner is tested against actual device output, not synthesized data.

- `both-streams.bin` — binary output (reg 75, port 2, 200 Hz, all 7
  Common fields = mask `0x0739`, 110 B/frame) **and** ASCII async
  (`$VNYMR`, 40 Hz) at once. ~269 kbit/s. Cleanly interleaved:
  intact frames and lines at message boundaries.
- `binary-only.bin` — binary output only, same config. ~220 kbit/s.
- `ascii-only.bin` — ASCII async `$VNYMR` @ 40 Hz only. ~49 kbit/s.

See the `feat: passive bench measures the live stream` section in
[chores-01.md](/notes/chores/chores-01.md) for context.
