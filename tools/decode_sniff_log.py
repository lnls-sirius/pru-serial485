#!/usr/bin/env python3
"""Decode or validate a PRUserial485 sniffer log file.

Usage:
    python3 decode_sniff_log.py <path-to-sniff_*.log> [more paths...]
    python3 decode_sniff_log.py --check <path-to-sniff_*.log> [more paths...]

Default mode dumps every record. Example:
    $ python3 decode_sniff_log.py sniff_logs/sniff_1785432000_123456789.log
    [     0] t=1785432000.123457  len=  22  0250001115...
    [     1] t=1785432000.134787  (+11.33ms)  len=  21  005100100000...
    [     2] t=1785432000.156201  (+21.41ms)  len=  22  ...
    3 messages decoded from sniff_logs/sniff_1785432000_123456789.log

    Multiple log files (e.g. after rotation) can be passed at once and are
    decoded in order:
    $ python3 decode_sniff_log.py sniff_logs/sniff_*.log

--check mode scans without dumping every record, meant for a huge log
file where reading a full dump isn't practical. It reports counts plus
three independent problem signals: any overflow/loss event the sniffer
itself detected (broken down by which of the two overflow conditions
produced it, since lost messages vs. lost bytes are different units and
are never summed together), any baseline-resync event (host_count found
ahead of pru_count, an anomaly of unknown cause, see sniffer.c), and
any gap in the per-message `seq` counter that ISN'T explained by either
of the above (a bug neither this script nor the sniffer anticipated,
since seq only advances on a successful write, and neither event type
skips a seq value). Example:
    $ python3 decode_sniff_log.py --check sniff_logs/sniff_*.log
    == sniff_logs/sniff_1785432000_123456789.log ==
      messages:              104213
      overflow events:       0
        lost messages (length-fifo entries overwritten): 0
        lost bytes (byte-ring underrun):                 0
      baseline resync events: 0
      unexplained gaps:      0
      no problems found
"""
import struct
import sys

HEADER_FMT = "<BBQII"   # magic, version, timestamp_ns, seq, length
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Each event type has its own magic byte, so classification never depends
# on interpreting bits within the count field.
MAGIC_MESSAGE = 0xA5
MAGIC_OVERFLOW_ENTRIES = 0xEE
MAGIC_OVERFLOW_BYTES = 0xEF
MAGIC_RESYNC = 0xED


def iter_records(path):
    """Yield one dict per record in path, in file order.

    kind is "message", "overflow", "resync", "truncated", or "unknown".
    Stops (without raising) after yielding a "truncated" or "unknown"
    record, since nothing past that point can be trusted to parse
    correctly.

    For "overflow" records, "reason" is "entries" or "bytes".
    """
    with open(path, "rb") as f:
        data = f.read()

    i = 0
    while i < len(data):
        if i + HEADER_SIZE > len(data):
            yield {"kind": "truncated", "offset": i, "trailing_bytes": len(data) - i}
            return

        record_offset = i
        magic, _fmt_version, ts_ns, seq, length = struct.unpack_from(HEADER_FMT, data, i)
        i += HEADER_SIZE

        if magic == MAGIC_MESSAGE:
            payload = data[i:i + length]
            i += length
            yield {"kind": "message", "seq": seq, "ts_ns": ts_ns,
                   "length": length, "payload": payload, "offset": record_offset}
        elif magic == MAGIC_RESYNC:
            yield {"kind": "resync", "seq": seq, "ts_ns": ts_ns,
                   "delta": length, "offset": record_offset}
        elif magic in (MAGIC_OVERFLOW_ENTRIES, MAGIC_OVERFLOW_BYTES):
            reason = "entries" if magic == MAGIC_OVERFLOW_ENTRIES else "bytes"
            yield {"kind": "overflow", "seq": seq, "ts_ns": ts_ns,
                   "count": length, "reason": reason, "offset": record_offset}
        else:
            yield {"kind": "unknown", "magic": magic, "offset": record_offset}
            return


def decode_file(path):
    last_ts = None
    count = 0
    for rec in iter_records(path):
        kind = rec["kind"]

        if kind == "truncated":
            print("  ...{} trailing bytes, incomplete record, stopping".format(
                rec["trailing_bytes"]))
            break
        if kind == "unknown":
            print("unknown magic {:#x} at offset {}, stopping".format(
                rec["magic"], rec["offset"]))
            break

        ts_s = rec["ts_ns"] / 1e9
        if last_ts is None:
            gap_ms = ""
        else:
            gap_ms = "  (+{:.2f}ms)".format((rec["ts_ns"] - last_ts) / 1e6)
        last_ts = rec["ts_ns"]

        if kind == "message":
            count += 1
            print("[{:6d}] t={:.6f}{}  len={:4d}  {}".format(
                rec["seq"], ts_s, gap_ms, rec["length"], rec["payload"].hex()))
        elif kind == "overflow":
            print("[{:6d}] t={:.6f}{}  ** OVERFLOW ({}) ** lost={}".format(
                rec["seq"], ts_s, gap_ms, rec["reason"], rec["count"]))
        elif kind == "resync":
            print("[{:6d}] t={:.6f}{}  ** BASELINE RESYNC ** delta={} "
                  "(exact loss, if any, unknown)".format(
                      rec["seq"], ts_s, gap_ms, rec["delta"]))

    print("{} messages decoded from {}".format(count, path))


def check_file(path):
    n_messages = 0
    n_overflow_events = 0
    n_lost_entries = 0     # length-fifo entries overwritten before being read
    n_lost_bytes = 0       # byte-ring underrun relative to length-fifo
    n_resync_events = 0    # host_count was ahead of pru_count, resynced
    n_seq_gaps = 0
    expected_seq = None
    problems = []

    for rec in iter_records(path):
        kind = rec["kind"]

        if kind == "truncated":
            problems.append(
                "truncated record at offset {} ({} trailing bytes), likely "
                "the process was killed or crashed mid-write"
                .format(rec["offset"], rec["trailing_bytes"]))
            break

        if kind == "unknown":
            problems.append(
                "unknown magic {:#x} at offset {}: file may be corrupted "
                "or this is not a sniffer log".format(rec["magic"], rec["offset"]))
            break

        if kind == "resync":
            n_resync_events += 1
            problems.append(
                "baseline resync at offset {} (delta={}): host_count was "
                "ahead of pru_count for an unexplained reason; exact loss, "
                "if any, unknown".format(rec["offset"], rec["delta"]))
            # Same reasoning as overflow events below: does not touch
            # expected_seq.
            continue

        if kind == "overflow":
            n_overflow_events += 1
            if rec["reason"] == "entries":
                n_lost_entries += rec["count"]
            else:
                n_lost_bytes += rec["count"]
            # Deliberately does NOT touch expected_seq: an overflow event
            # repeats the current seq rather than skipping it (see
            # sniffer.c), so it can never legitimately explain a gap
            # between two message records' seq values.
            continue

        # kind == "message"
        n_messages += 1
        if expected_seq is not None and rec["seq"] != expected_seq:
            n_seq_gaps += 1
            problems.append(
                "seq jumped from {} to {} at offset {} (gap of {}) with NO "
                "overflow event logged for it: unexplained, possibly a "
                "bug this sniffer build doesn't detect"
                .format(expected_seq, rec["seq"], rec["offset"],
                        rec["seq"] - expected_seq))
        expected_seq = rec["seq"] + 1

    print("== {} ==".format(path))
    print("  messages:              {}".format(n_messages))
    print("  overflow events:       {}".format(n_overflow_events))
    print("    lost messages (length-fifo entries overwritten): {}".format(n_lost_entries))
    print("    lost bytes (byte-ring underrun):                 {}".format(n_lost_bytes))
    print("  baseline resync events: {}".format(n_resync_events))
    print("  unexplained gaps:      {}".format(n_seq_gaps))
    if problems:
        print("  PROBLEMS FOUND:")
        for p in problems:
            print("    - {}".format(p))
    else:
        print("  no problems found")
    print("")


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(1)

    if args[0] == "--check":
        paths = args[1:]
        if not paths:
            print(__doc__)
            sys.exit(1)
        for path in paths:
            check_file(path)
    else:
        for path in args:
            decode_file(path)
