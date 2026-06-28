#!/usr/bin/env bash
# Reproduce the passive-bench zero-parse — a full byte read off the
# wire that parses into zero messages/frames (Todo #1; analysis in
# notes/chores/chores-01.md "Zero-parse root cause: PL011 baud-change
# open").
#
# Mechanism: on the RPi5 PL011 UART, a fresh open whose baud differs
# from the previous open corrupts the read two ways — sometimes the
# new divisor never applies (stale, undersampled garbage), sometimes
# framing stays intact but bit 6 (B6) flips on scattered bytes so
# every CRC fails. So after putting the device on 921600, we make
# every measured open a baud *change*: a wrong-baud `--baud 115200`
# open immediately before each captured `--baud 921600` read. ~20%
# of those opens fail, the device never changes, and the next open
# always parses clean.
#
# This script configures the device itself (volatile — a power cycle
# reverts to the flash 115200) and leaves it on 921600 + a heavy
# 7-field stream.
#
# Usage: repro.sh [OUT_DIR] [N] [START_BAUD]
#   OUT_DIR    capture directory (default ./tmp)
#   N          stress iterations (default 20)
#   START_BAUD device's current baud (default 115200, the flash
#              default; pass 921600 to re-run on an already-switched
#              device)
set -u
OUT=${1:-./tmp}
N=${2:-20}
START_BAUD=${3:-115200}
mkdir -p "$OUT"

# 1. Put the device on 921600 with a heavy 7-field binary + YMR
#    stream. Each step opens at 921600 after the switch, leaving the
#    host PL011 divisor there.
echo "configuring device: ${START_BAUD} -> 921600, heavy 7-field + YMR..."
rw-vn100 baud 921600 --baud "$START_BAUD" || {
  echo "baud switch failed — is the device at ${START_BAUD}?"
  echo "(re-run with START_BAUD=921600, or power-cycle the device)"
  exit 1
}
rw-vn100 set-ascii=off --baud 921600
rw-vn100 set-bin=off --baud 921600
rw-vn100 set-bin-fields=time,ypr,quat,gyro,accel,imu,magpres --baud 921600
rw-vn100 set-bin-hz=200 --baud 921600
rw-vn100 set-bin=on --baud 921600
rw-vn100 set-ascii=ymr --baud 921600

# 2. Sanity check: a same-baud 921600 read must parse before we start
#    the baud-change stress, or the test means nothing.
echo "sanity check (clean 921600 read):"
rw-vn100 bench 1 --baud 921600 || {
  echo "device not streaming cleanly at 921600 — aborting"
  exit 1
}

# 3. Stress loop: each captured 921600 read is preceded by a
#    now-genuinely-wrong-baud 115200 open (device is at 921600), so
#    every measured open is a baud change.
echo "stress loop (${N}x baud-change opens):"
for i in $(seq 1 "$N"); do
  rw-vn100 bench 1 --baud 115200 >/dev/null 2>&1   # wrong-baud flip
  ts=$(date +%H%M%S%3N)
  printf "run %2d: " "$i"
  rw-vn100 bench 1 --baud 921600 --capture "$OUT/cap-alt-r${i}-${ts}.bin" 2>&1 \
    | grep -E "ASCII:|Binary:|none|error" | tr '\n' ' '
  echo
done
