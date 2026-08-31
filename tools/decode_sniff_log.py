#!/usr/bin/env python3
"""Decode or validate a PRUserial485 sniffer log file.

Usage:
    python3 decode_sniff_log.py <path-to-sniff_*.log> [more paths...]
    python3 decode_sniff_log.py --check <path-to-sniff_*.log> [more paths...]
    python3 decode_sniff_log.py --bsmp <path-to-sniff_*.log> [more paths...]
    python3 decode_sniff_log.py --bsmp-resync [--full-garbage] [--commands=standard|0x12,0x13,...] <path-to-sniff_*.log> [more paths...]

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
five independent problem signals: any overflow/loss event the sniffer
itself detected (broken down by which of the two overflow conditions
produced it, since lost messages vs. lost bytes are different units and
are never summed together), any baseline-resync event (host_count found
ahead of pru_count, an anomaly of unknown cause, see sniffer.c), any
bad-length event (the capture thread found an implausible length-fifo
entry, a logic error somewhere upstream and not expected in normal
operation, and discarded the rest of that batch rather than risk
misaligned slicing silently corrupting message framing; see sniffer.c),
any writer-queue-overflow event (the capture thread's own record was
formed correctly but the in-memory queue to the disk-writer thread was
full, e.g. because storage couldn't keep up with a sustained burst, so
it was dropped before ever reaching sniffer_seq; see sniffer.c), and any
gap in the per-message `seq` counter that ISN'T explained by any of the
above (a bug neither this script nor the sniffer anticipated, since seq
only advances on a successful write, and none of the other event types
skip a seq value). Example:
    $ python3 decode_sniff_log.py --check sniff_logs/sniff_*.log
    == sniff_logs/sniff_1785432000_123456789.log ==
      messages:              104213
      overflow events:       0
        lost messages (length-fifo entries overwritten): 0
        lost bytes (byte-ring underrun):                 0
      baseline resync events: 0
      bad-length events:      0
      writer queue overflow events: 0
        record bytes dropped before reaching disk: 0
      unexplained gaps:      0
      no problems found

--bsmp mode re-splits each record's raw payload into actual BSMP wire
frames instead of trusting the sniffer's own record boundary to mean
one protocol message. That boundary is just "PRU1 saw >= RXTIMEOUT
character-times of bus silence"; at a high RXTIMEOUT (e.g. 0x10) a fast
command/reply turnaround easily lands well inside that window, so one
record commonly contains two or more concatenated frames. The wire
format (fbp/udc-arm-fw's rs485.c) is the same in both directions:
    [address(1)] [command(1)] [size_hi(1)] [size_lo(1)] [payload(size)] [checksum(1)]
where the sum of every byte in the frame (address..checksum inclusive)
must equal 0 mod 256. Address 0 always means a UDC-to-master reply;
1-32 addresses a specific UDC; 255 is a master broadcast. This mode
walks each record's bytes accepting only checksum-valid frames at that
format, and reports whatever's left over (if anything) separately
rather than guessing past it. Example:
    $ python3 decode_sniff_log.py --bsmp sniff_logs/sniff_....log
    [     4] t=1785432000.171623  (+15.42ms)  len= 411  2 bsmp frame(s)
        frame[0] off=  0  CMD   addr=  2  cmd=0x12 CMD_GROUP_READ       size=  1  csum=0xe8 OK  payload=03
        frame[1] off=  6  REPLY addr=  0  cmd=0x13 CMD_GROUP_VALUES     size=390 csum=0x7a OK  payload=...
    3 messages / 5 bsmp frame(s) decoded from sniff_logs/sniff_....log

--bsmp-resync mode is for a byte stream too garbled for --bsmp's strict
per-record parse to get anywhere (e.g. a capture predating Passive mode,
where the sniffer's old "Sending Data" region sharing bug let unrelated
writes corrupt what it captured). Rather than trusting record
boundaries or bailing at the first invalid frame, it scans every byte
position looking for something that plausibly IS a frame: a real bus
address, a size within what this bus can physically produce, and a
valid checksum, all three at once. It resyncs one byte at a time past
anything that fails, so it can recover frames sitting at unaligned
offsets or preceded by garbage - but that does NOT mean it can never
match a false positive at random (~1 in 2000 by construction); this is
a best-effort read on data already known to be unreliable, not a
correctness guarantee. It never spans a real data-loss event (overflow/
resync/bad-length): the byte stream genuinely breaks there. GARBAGE
spans print at most 32 hex bytes by default (a badly garbled file can
have a lot of them); pass --full-garbage to print every byte instead -
the reported length and the final garbage-byte-count are always exact
either way, only the hex dump is capped.

--commands=0x12,0x13,0x50,0x51 adds a fourth, optional filter: only
accept a candidate whose command byte is in this list (hex or decimal,
comma-separated). Worth using specifically against command 0x00
(CMD_QUERY_VERSION) flooding: address=0, command=0, size=0, checksum=0
is five zero bytes, and zero is by far the most common byte value in
real garbage (float padding, uninitialized RAM), so any run of >=5
zero bytes anywhere produces an unbroken chain of degenerate "frames"
the other three filters cannot distinguish from a real one - they
reject implausible bytes, not implausible context. Restricting to the
commands actually expected on your bus routes a whole zero-run back
into one garbage span instead. --commands=standard is shorthand for
CMD_GROUP_READ, CMD_GROUP_VALUES, CMD_FUNC_EXECUTE, CMD_FUNC_RETURN
(0x12, 0x13, 0x50, 0x51) - the actual steady-state traffic on this bus
(BSMP_RESYNC_STANDARD_COMMANDS). Example:
    $ python3 decode_sniff_log.py --bsmp-resync sniff_logs/sniff_....log
      [near seq   142, t=1787054620.545178] CMD   addr=  2  cmd=0x50 CMD_FUNC_EXECUTE      size=  17  csum=0x2c OK  payload=15...
      [near seq   142, t=1787054620.545178] ** GARBAGE ** 31 byte(s)  data=4060914c... ...(15 more)
    12 plausible bsmp frame(s), 890 garbage byte(s) skipped, from sniff_logs/sniff_....log
"""
import bisect
import struct
import sys

# enum command_code, libbsmp/src/bsmp_priv.h
BSMP_COMMAND_NAMES = {
    0x00: "CMD_QUERY_VERSION",
    0x01: "CMD_VERSION",
    0x02: "CMD_VAR_QUERY_LIST",
    0x03: "CMD_VAR_LIST",
    0x04: "CMD_GROUP_QUERY_LIST",
    0x05: "CMD_GROUP_LIST",
    0x06: "CMD_GROUP_QUERY",
    0x07: "CMD_GROUP",
    0x08: "CMD_CURVE_QUERY_LIST",
    0x09: "CMD_CURVE_LIST",
    0x0A: "CMD_CURVE_QUERY_CSUM",
    0x0B: "CMD_CURVE_CSUM",
    0x0C: "CMD_FUNC_QUERY_LIST",
    0x0D: "CMD_FUNC_LIST",
    0x10: "CMD_VAR_READ",
    0x11: "CMD_VAR_VALUE",
    0x12: "CMD_GROUP_READ",
    0x13: "CMD_GROUP_VALUES",
    0x20: "CMD_VAR_WRITE",
    0x22: "CMD_GROUP_WRITE",
    0x24: "CMD_VAR_BIN_OP",
    0x26: "CMD_GROUP_BIN_OP",
    0x28: "CMD_VAR_WRITE_READ",
    0x30: "CMD_GROUP_CREATE",
    0x32: "CMD_GROUP_REMOVE_ALL",
    0x40: "CMD_CURVE_BLOCK_REQUEST",
    0x41: "CMD_CURVE_BLOCK",
    0x42: "CMD_CURVE_RECALC_CSUM",
    0x50: "CMD_FUNC_EXECUTE",
    0x51: "CMD_FUNC_RETURN",
    0x53: "CMD_FUNC_ERROR",
    0xE0: "CMD_OK",
    0xE1: "CMD_ERR_MALFORMED_MESSAGE",
    0xE2: "CMD_ERR_OP_NOT_SUPPORTED",
    0xE3: "CMD_ERR_INVALID_ID",
    0xE4: "CMD_ERR_INVALID_VALUE",
    0xE5: "CMD_ERR_INVALID_PAYLOAD_SIZE",
    0xE6: "CMD_ERR_READ_ONLY",
    0xE7: "CMD_ERR_INSUFFICIENT_MEMORY",
    0xE8: "CMD_ERR_RESOURCE_BUSY",
}

BSMP_BROADCAST_ADDRESS = 255
BSMP_REPLY_ADDRESS = 0

# Frame header is address+command+size_hi+size_lo; anything shorter can
# never even be inspected for a declared size.
BSMP_FRAME_HEADER_BYTES = 4

# For --bsmp-resync's plausibility filter, not split_bsmp_frames()'s
# strict path: the largest frame this bus can physically produce, same
# bound sniffer.c itself uses (SNIFFER_MAX_PLAUSIBLE_MSG_LEN) to reject
# implausible length-fifo entries, and the valid RS-485 node addresses
# (specific UDC 1-32, broadcast 255, or 0 for a UDC-to-master reply).
BSMP_RESYNC_MAX_SIZE = 4093
BSMP_RESYNC_PLAUSIBLE_ADDRESSES = frozenset([0, BSMP_BROADCAST_ADDRESS]) | frozenset(range(1, 33))

# --commands=standard: the steady-state traffic this bus actually carries
# (feedforward's CMD_FUNC_EXECUTE/CMD_FUNC_RETURN loop plus supervisory
# CMD_GROUP_READ/CMD_GROUP_VALUES polling). Excluding CMD_QUERY_VERSION
# (0x00) in particular is what matters: see resync_scan_frames()'s
# command_allowlist docstring for why that one code floods results.
BSMP_RESYNC_STANDARD_COMMANDS = frozenset([0x12, 0x13, 0x50, 0x51])

# Cap how much of a garbage span's hex gets printed, so a badly garbled
# file doesn't flood the terminal with bytes nobody's going to read.
BSMP_RESYNC_GARBAGE_HEX_CAP = 32

# magic, version, timestamp_ns, seq, length
HEADER_FMT = "<BBQII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Each event type has its own magic byte, so classification never depends
# on interpreting bits within the count field.
MAGIC_MESSAGE = 0xA5
MAGIC_OVERFLOW_ENTRIES = 0xEE
MAGIC_OVERFLOW_BYTES = 0xEF
MAGIC_RESYNC = 0xED
MAGIC_BAD_LENGTH = 0xEC
MAGIC_QUEUE_OVERFLOW = 0xEB


def iter_records(path):
    """Yield one dict per record in path, in file order.

    kind is "message", "overflow", "resync", "bad_length",
    "queue_overflow", "truncated", or "unknown". Stops (without raising)
    after yielding a "truncated" or "unknown" record, since nothing past
    that point can be trusted to parse correctly.

    For "overflow" records, "reason" is "entries" or "bytes". For
    "queue_overflow" records, "dropped_bytes" is a count of raw record
    bytes (message and/or event records alike) dropped before reaching
    disk; it never affects "seq" continuity, since a dropped message
    never consumes a seq value in the first place (see sniffer.c).
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
        elif magic == MAGIC_BAD_LENGTH:
            yield {"kind": "bad_length", "seq": seq, "ts_ns": ts_ns,
                   "bad_length": length, "offset": record_offset}
        elif magic in (MAGIC_OVERFLOW_ENTRIES, MAGIC_OVERFLOW_BYTES):
            reason = "entries" if magic == MAGIC_OVERFLOW_ENTRIES else "bytes"
            yield {"kind": "overflow", "seq": seq, "ts_ns": ts_ns,
                   "count": length, "reason": reason, "offset": record_offset}
        elif magic == MAGIC_QUEUE_OVERFLOW:
            yield {"kind": "queue_overflow", "seq": seq, "ts_ns": ts_ns,
                   "dropped_bytes": length, "offset": record_offset}
        else:
            yield {"kind": "unknown", "magic": magic, "offset": record_offset}
            return


def bsmp_command_name(code):
    name = BSMP_COMMAND_NAMES.get(code)
    return "{} {}".format("{:#04x}".format(code), name) if name else "{:#04x} ?".format(code)


def bsmp_direction(address):
    if address == BSMP_REPLY_ADDRESS:
        return "REPLY"
    if address == BSMP_BROADCAST_ADDRESS:
        return "BCAST"
    return "CMD"


def split_bsmp_frames(payload):
    """Split one sniffer record's raw payload into BSMP wire frames.

    Wire format (fbp/udc-arm-fw's rs485.c, identical for commands and
    replies): address(1) command(1) size_hi(1) size_lo(1) payload(size)
    checksum(1); the sum of every byte in the frame, checksum included,
    must equal 0 mod 256. Total frame length is size+5.

    Yields dicts with kind "frame" (offset, address, command, size,
    payload) for each checksum-valid frame found, in order. Stops and
    yields one final kind "leftover" (offset, data, reason) dict if the
    remaining bytes can't form a complete, checksum-valid frame -
    that's expected at the very end of a record only when a data-loss
    event (overflow/resync/bad-length) cut it short; anywhere else it
    means the two sides of this "boundary" were never one bsmp exchange
    to begin with.
    """
    offset = 0
    n = len(payload)
    while offset < n:
        if offset + BSMP_FRAME_HEADER_BYTES > n:
            yield {"kind": "leftover", "offset": offset, "data": payload[offset:],
                   "reason": "only {} byte(s) left, fewer than the {}-byte frame header"
                             .format(n - offset, BSMP_FRAME_HEADER_BYTES)}
            return

        address = payload[offset]
        command = payload[offset + 1]
        size = (payload[offset + 2] << 8) | payload[offset + 3]
        frame_len = size + 5

        if offset + frame_len > n:
            yield {"kind": "leftover", "offset": offset, "data": payload[offset:],
                   "reason": "declared size {} needs {} byte(s), only {} remain"
                             .format(size, frame_len, n - offset)}
            return

        frame_bytes = payload[offset:offset + frame_len]
        checksum = sum(frame_bytes) & 0xFF
        if checksum != 0:
            yield {"kind": "leftover", "offset": offset, "data": payload[offset:],
                   "reason": "checksum mismatch at declared size {} (sum={:#04x}, want 0x00)"
                             .format(size, checksum)}
            return

        yield {"kind": "frame", "offset": offset, "address": address,
               "command": command, "size": size,
               "payload": payload[offset + 4:offset + 4 + size],
               "checksum": frame_bytes[-1]}
        offset += frame_len


def resync_scan_frames(buf, command_allowlist=None):
    """Best-effort scan of a raw byte buffer for BSMP frames, resyncing
    byte-by-byte past anything that doesn't look like one.

    Unlike split_bsmp_frames(), this never trusts a starting offset to
    already be the start of a frame: it tries the wire format (see
    split_bsmp_frames' docstring) at every byte position in turn. A
    candidate is accepted only if its address is one that could
    plausibly appear on this bus, its declared size doesn't exceed the
    largest frame this bus can physically produce, AND its checksum
    validates - three independent, cheap filters that together make a
    false accept rare (roughly 1 in 2000 random positions) but not
    impossible. This is a best-effort realignment for a byte stream
    that's already known to be garbled (e.g. corrupted by the pre-
    Passive-mode "Sending Data" sharing bug), not a guarantee that every
    frame reported is genuine - read results with that in mind.

    command_allowlist, if given, adds a fourth filter: only accept a
    candidate whose command byte is in this set. Worth using when
    command 0x00 (CMD_QUERY_VERSION) in particular is flooding the
    output: address=0, command=0, size=0, checksum=0 is five zero bytes,
    and zero is by far the most common byte value in real garbage
    (float padding, uninitialized RAM, etc.), so any run of >=5 zero
    bytes anywhere produces an unbroken chain of degenerate "frames"
    that the other three filters cannot tell apart from a real one -
    they were never designed to reject a technically-valid, just
    contextually-implausible, all-zero frame. Restricting to the
    command codes actually expected on this bus (e.g. only
    CMD_GROUP_READ/CMD_GROUP_VALUES/CMD_FUNC_EXECUTE/CMD_FUNC_RETURN)
    routes that whole zero-run back into one "garbage" span instead.

    Yields dicts: kind "frame" (offset, address, command, size, payload)
    for each accepted frame, or kind "garbage" (offset, length) for each
    contiguous span of rejected bytes in between.
    """
    n = len(buf)
    pos = 0
    garbage_start = None

    while pos < n:
        if pos + BSMP_FRAME_HEADER_BYTES > n:
            break

        address = buf[pos]
        command = buf[pos + 1]
        size = (buf[pos + 2] << 8) | buf[pos + 3]
        frame_len = size + 5

        accepted = (
            address in BSMP_RESYNC_PLAUSIBLE_ADDRESSES
            and size <= BSMP_RESYNC_MAX_SIZE
            and (command_allowlist is None or command in command_allowlist)
            and pos + frame_len <= n
            and sum(buf[pos:pos + frame_len]) & 0xFF == 0
        )

        if accepted:
            if garbage_start is not None:
                yield {"kind": "garbage", "offset": garbage_start, "length": pos - garbage_start}
                garbage_start = None
            yield {"kind": "frame", "offset": pos, "address": address,
                   "command": command, "size": size,
                   "payload": bytes(buf[pos + 4:pos + 4 + size]),
                   "checksum": buf[pos + frame_len - 1]}
            pos += frame_len
        else:
            if garbage_start is None:
                garbage_start = pos
            pos += 1

    if garbage_start is not None:
        yield {"kind": "garbage", "offset": garbage_start, "length": n - garbage_start}
    elif pos < n:
        # trailing bytes too short to even try a header against
        yield {"kind": "garbage", "offset": pos, "length": n - pos}


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
        elif kind == "bad_length":
            print("[{:6d}] t={:.6f}{}  ** IMPLAUSIBLE LENGTH ** value={} "
                  "(rest of that batch discarded, exact loss, if any, unknown)".format(
                      rec["seq"], ts_s, gap_ms, rec["bad_length"]))
        elif kind == "queue_overflow":
            print("[{:6d}] t={:.6f}{}  ** WRITER QUEUE OVERFLOW ** "
                  "dropped_bytes={} (disk couldn't keep up, see sniffer.c)".format(
                      rec["seq"], ts_s, gap_ms, rec["dropped_bytes"]))

    print("{} messages decoded from {}".format(count, path))


def dump_bsmp(path):
    last_ts = None
    n_messages = 0
    n_frames = 0
    n_leftovers = 0

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
            n_messages += 1
            frames = list(split_bsmp_frames(rec["payload"]))
            n_frame_parts = sum(1 for f in frames if f["kind"] == "frame")
            print("[{:6d}] t={:.6f}{}  len={:4d}  {} bsmp frame(s)".format(
                rec["seq"], ts_s, gap_ms, rec["length"], n_frame_parts))
            for idx, f in enumerate(frames):
                if f["kind"] == "frame":
                    n_frames += 1
                    print("    frame[{}] off={:4d}  {:5s} addr={:3d}  cmd={:26s}  "
                          "size={:4d}  csum={:#04x} OK  payload={}".format(
                              idx, f["offset"], bsmp_direction(f["address"]),
                              f["address"], bsmp_command_name(f["command"]),
                              f["size"], f["checksum"], f["payload"].hex()))
                else:
                    n_leftovers += 1
                    print("    frame[{}] off={:4d}  ** LEFTOVER ** {}  data={}".format(
                        idx, f["offset"], f["reason"], f["data"].hex()))
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
        elif kind == "queue_overflow":
            print("[{:6d}] t={:.6f}{}  ** WRITER QUEUE OVERFLOW ** "
                  "dropped_bytes={} (disk couldn't keep up, see sniffer.c)".format(
                      rec["seq"], ts_s, gap_ms, rec["dropped_bytes"]))

    print("{} messages / {} bsmp frame(s) decoded from {}{}".format(
        n_messages, n_frames, path,
        "" if n_leftovers == 0 else " ({} leftover span(s), see above)".format(n_leftovers)))


def _flush_resync_segment(segment_bytes, segment_spans, n_frames, n_garbage_bytes,
                           garbage_hex_cap=BSMP_RESYNC_GARBAGE_HEX_CAP, command_allowlist=None):
    """Scan one contiguous (no known data-loss) span of message bytes and
    print whatever resync_scan_frames() finds, attributing each result
    back to the original record it started in for an approximate
    timestamp - approximate because a frame or garbage span can extend
    past that record's own bytes into the one(s) captured after it,
    which is exactly the merged-record case this mode exists for.
    """
    if not segment_bytes:
        return n_frames, n_garbage_bytes

    span_starts = [s[0] for s in segment_spans]

    def attribute(offset, end):
        idx = bisect.bisect_right(span_starts, offset) - 1
        start, stop, seq, ts_ns = segment_spans[idx]
        note = "" if end <= stop else "  (extends into later record(s))"
        return seq, ts_ns, note

    for item in resync_scan_frames(segment_bytes, command_allowlist):
        if item["kind"] == "frame":
            n_frames += 1
            end = item["offset"] + 5 + item["size"]
            seq, ts_ns, note = attribute(item["offset"], end)
            print("  [near seq {:6d}, t={:.6f}] {:5s} addr={:3d}  cmd={:26s}  "
                  "size={:4d}  csum={:#04x} OK  payload={}{}".format(
                      seq, ts_ns / 1e9, bsmp_direction(item["address"]),
                      item["address"], bsmp_command_name(item["command"]),
                      item["size"], item["checksum"], item["payload"].hex(), note))
        else:
            n_garbage_bytes += item["length"]
            seq, ts_ns, note = attribute(item["offset"], item["offset"] + item["length"])
            if garbage_hex_cap is None:
                data = segment_bytes[item["offset"]:item["offset"] + item["length"]]
                trailer = ""
            else:
                data = segment_bytes[item["offset"]:item["offset"] + garbage_hex_cap]
                trailer = "" if item["length"] <= garbage_hex_cap else \
                    " ...({} more)".format(item["length"] - garbage_hex_cap)
            print("  [near seq {:6d}, t={:.6f}] ** GARBAGE ** {} byte(s){}  data={}{}".format(
                seq, ts_ns / 1e9, item["length"], note, bytes(data).hex(), trailer))

    return n_frames, n_garbage_bytes


def dump_bsmp_resync(path, garbage_hex_cap=BSMP_RESYNC_GARBAGE_HEX_CAP, command_allowlist=None):
    segment_bytes = bytearray()
    segment_spans = []  # (start_offset_in_segment, end_offset, seq, ts_ns)
    n_frames = 0
    n_garbage_bytes = 0

    for rec in iter_records(path):
        kind = rec["kind"]

        if kind == "message":
            start = len(segment_bytes)
            segment_bytes.extend(rec["payload"])
            segment_spans.append((start, len(segment_bytes), rec["seq"], rec["ts_ns"]))
            continue

        # Anything else is a genuine break in the byte stream: flush what
        # we have (nothing before this point can span across it), report
        # the event itself exactly as --check/default mode would, then
        # start a fresh segment.
        n_frames, n_garbage_bytes = _flush_resync_segment(
            segment_bytes, segment_spans, n_frames, n_garbage_bytes, garbage_hex_cap, command_allowlist)
        segment_bytes = bytearray()
        segment_spans = []

        if kind == "overflow":
            print("-- OVERFLOW ({}) at seq {}: lost={}, byte stream breaks here --".format(
                rec["reason"], rec["seq"], rec["count"]))
        elif kind == "resync":
            print("-- BASELINE RESYNC at seq {}: delta={}, byte stream breaks here --".format(
                rec["seq"], rec["delta"]))
        elif kind == "bad_length":
            print("-- IMPLAUSIBLE LENGTH at seq {}: value={}, byte stream breaks here --".format(
                rec["seq"], rec["bad_length"]))
        elif kind == "queue_overflow":
            print("-- WRITER QUEUE OVERFLOW reported near seq {}: dropped_bytes={}, "
                  "byte stream breaks somewhere before here (exact point unknown, "
                  "drops are only reported once the writer catches up) --".format(
                      rec["seq"], rec["dropped_bytes"]))
        elif kind == "truncated":
            print("  ...{} trailing bytes, incomplete record, stopping".format(
                rec["trailing_bytes"]))
            break
        elif kind == "unknown":
            print("unknown magic {:#x} at offset {}, stopping".format(
                rec["magic"], rec["offset"]))
            break

    n_frames, n_garbage_bytes = _flush_resync_segment(
        segment_bytes, segment_spans, n_frames, n_garbage_bytes, garbage_hex_cap, command_allowlist)

    print("{} plausible bsmp frame(s), {} garbage byte(s) skipped, from {}".format(
        n_frames, n_garbage_bytes, path))


def check_file(path):
    n_messages = 0
    n_overflow_events = 0

    # length-fifo entries overwritten before being read
    n_lost_entries = 0

    # byte-ring underrun relative to length-fifo
    n_lost_bytes = 0

    # host_count was ahead of pru_count, resynced
    n_resync_events = 0

    # implausible length-fifo entry, batch discarded
    n_bad_length_events = 0

    # writer thread's in-memory queue was full, record(s) dropped before
    # reaching disk
    n_queue_overflow_events = 0
    n_queue_overflow_bytes = 0

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

        if kind == "queue_overflow":
            n_queue_overflow_events += 1
            n_queue_overflow_bytes += rec["dropped_bytes"]
            problems.append(
                "writer queue overflow at offset {} (dropped_bytes={}): the "
                "writer thread's in-memory queue was full, so record(s) were "
                "dropped before reaching disk; storage likely can't keep up "
                "with sustained traffic, see sniffer.c".format(
                    rec["offset"], rec["dropped_bytes"]))
            # Also does NOT touch expected_seq: a dropped message never
            # allocates a seq value in the first place (see sniffer.c), so
            # this can never legitimately explain a gap either.
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
    print("  bad-length events:      {}".format(n_bad_length_events))
    print("  writer queue overflow events: {}".format(n_queue_overflow_events))
    print("    record bytes dropped before reaching disk: {}".format(n_queue_overflow_bytes))
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
    elif args[0] == "--bsmp":
        paths = args[1:]
        if not paths:
            print(__doc__)
            sys.exit(1)
        for path in paths:
            dump_bsmp(path)
    elif args[0] == "--bsmp-resync":
        rest = args[1:]
        full_garbage = "--full-garbage" in rest
        rest = [a for a in rest if a != "--full-garbage"]
        allowlist = None
        paths = []
        for a in rest:
            if a.startswith("--commands="):
                spec = a[len("--commands="):]
                if spec == "standard":
                    allowlist = BSMP_RESYNC_STANDARD_COMMANDS
                else:
                    allowlist = frozenset(int(c, 0) for c in spec.split(","))
            else:
                paths.append(a)
        if not paths:
            print(__doc__)
            sys.exit(1)
        cap = None if full_garbage else BSMP_RESYNC_GARBAGE_HEX_CAP
        for path in paths:
            dump_bsmp_resync(path, garbage_hex_cap=cap, command_allowlist=allowlist)
    else:
        for path in args:
            decode_file(path)
