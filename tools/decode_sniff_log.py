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

A message record can also be a forced chunk boundary rather than a real
message: PRUserial485.p's byte-count safety valve (FORCE_FLUSH_SLAVE)
cuts a message boundary at FORCE_FLUSH_THRESHOLD bytes whenever the bus
never goes idle long enough for RxTimeout to find a real one. Chunk
records are real, byte-exact traffic, but NOT a real protocol message
boundary, so they're marked distinctly rather than presented as if they
were an ordinary message; see sniffer.c.

--check mode scans without dumping every record, meant for a huge log
file where reading a full dump isn't practical. It reports counts plus
five independent problem signals: any overflow/loss event the sniffer
itself detected (broken down by which of the two overflow conditions
produced it, since lost messages vs. lost bytes are different units and
are never summed together), any baseline-resync event (host_count found
ahead of pru_count, an anomaly of unknown cause, see sniffer.c), any
bad-length event (the capture thread found an implausible length-fifo
entry, a logic error somewhere upstream and not expected in normal
operation, and discarded the rest of that batch rather than risk
misaligned slicing silently corrupting message framing; see sniffer.c),
any gap in the per-message `seq` counter that ISN'T explained by any of
the above (a bug neither this script nor the sniffer anticipated, since
seq only advances on a successful write, and none of the other event
types skip a seq value), and a separate count of chunked messages (not
a problem by itself, but a high count means the bus spent a lot of time
genuinely gapless, worth knowing even on an otherwise clean capture).
Example:
    $ python3 decode_sniff_log.py --check sniff_logs/sniff_*.log
    == sniff_logs/sniff_1785432000_123456789.log ==
      messages:              104213
      chunked messages:      312
      overflow events:       0
        lost messages (length-fifo entries overwritten): 0
        lost bytes (byte-ring underrun):                 0
      baseline resync events: 0
      bad-length events:      0
      unexplained gaps:      0
      no problems found
"""
import struct
import sys

# magic, version, timestamp_ns, seq, length
HEADER_FMT = "<BBQII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Each event type has its own magic byte, so classification never depends
# on interpreting bits within the count field.
MAGIC_MESSAGE = 0xA5
MAGIC_MESSAGE_CHUNK = 0xA6
MAGIC_OVERFLOW_ENTRIES = 0xEE
MAGIC_OVERFLOW_BYTES = 0xEF
MAGIC_RESYNC = 0xED
MAGIC_BAD_LENGTH = 0xEC


def iter_records(path):
    """Yield one dict per record in path, in file order.

    kind is "message", "overflow", "resync", "truncated", or "unknown".
    Stops (without raising) after yielding a "truncated" or "unknown"
    record, since nothing past that point can be trusted to parse
    correctly.

    For "message" records, "chunked" is True when the PRU's byte-count
    safety valve forced this boundary (FORCE_FLUSH_SLAVE in
    PRUserial485.p) rather than a real RxTimeout completion: real,
    byte-exact traffic, but not an actual protocol message boundary.

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

        if magic == MAGIC_MESSAGE or magic == MAGIC_MESSAGE_CHUNK:
            payload = data[i:i + length]
            i += length
            yield {"kind": "message", "chunked": magic == MAGIC_MESSAGE_CHUNK,
                   "seq": seq, "ts_ns": ts_ns,
                   "length": length, "payload": payload, "offset": record_offset}
        elif magic == MAGIC_RESYNC:
            yield {"kind": "resync", "seq": seq, "ts_ns": ts_ns,
                   "delta": length, "offset": record_offset}
        elif magic == MAGIC_BAD_LENGTH:
            yield {"kind": "bad_length", "seq": seq, "ts_ns": ts_ns,
                   "bad_length": length, "offset": record_offset}
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
    chunk_count = 0
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
            if rec["chunked"]:
                chunk_count += 1
                print("[{:6d}] t={:.6f}{}  ** CHUNK (forced flush, not a real "
                      "message boundary) **  len={:4d}  {}".format(
                          rec["seq"], ts_s, gap_ms, rec["length"], rec["payload"].hex()))
            else:
                print("[{:6d}] t={:.6f}{}  len={:4d}  {}".format(
                    rec["seq"], ts_s, gap_ms, rec["length"], rec["payload"].hex()))
        elif kind == "overflow":
            print("[{:6d}] t={:.6f}{}  ** OVERFLOW ({}) ** lost={}".format(
                rec["seq"], ts_s, gap_ms, rec["reason"], rec["count"]))
        elif kind == "resync":
            print("[{:6d}] t={:.6f}{}  ** BASELINE RESYNC ** delta={} "
                  "(exact loss, if any, unknown)".format(
                      rec["seq"], ts_s, gap_ms, rec["delta"]))
        elif kind == "bad_length":
            print("[{:6d}] t={:.6f}{}  ** IMPLAUSIBLE LENGTH ** value={} "
                  "(rest of that batch discarded, exact loss, if any, unknown)".format(
                      rec["seq"], ts_s, gap_ms, rec["bad_length"]))

    print("{} messages decoded from {} ({} chunked)".format(count, path, chunk_count))


def check_file(path):
    n_messages = 0
    n_chunked_messages = 0
    n_overflow_events = 0

    # length-fifo entries overwritten before being read
    n_lost_entries = 0

    # byte-ring underrun relative to length-fifo
    n_lost_bytes = 0

    # host_count was ahead of pru_count, resynced
    n_resync_events = 0

    # implausible length-fifo entry, batch discarded
    n_bad_length_events = 0
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

        if kind == "bad_length":
            n_bad_length_events += 1
            problems.append(
                "implausible length-fifo entry at offset {} (value={}): "
                "rest of that batch was discarded before it could corrupt "
                "message framing; exact loss, if any, unknown"
                .format(rec["offset"], rec["bad_length"]))
            # Same reasoning as overflow/resync: does not touch expected_seq.
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

        # kind == "message" (real or chunked, both advance seq the same
        # way, see sniffer_log_write_message_ex() in sniffer.c, so both
        # participate in the gap check below identically)
        n_messages += 1
        if rec["chunked"]:
            n_chunked_messages += 1
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
    print("  chunked messages:      {}".format(n_chunked_messages))
    print("  overflow events:       {}".format(n_overflow_events))
    print("    lost messages (length-fifo entries overwritten): {}".format(n_lost_entries))
    print("    lost bytes (byte-ring underrun):                 {}".format(n_lost_bytes))
    print("  baseline resync events: {}".format(n_resync_events))
    print("  bad-length events:      {}".format(n_bad_length_events))
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
