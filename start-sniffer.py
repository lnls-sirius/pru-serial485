import PRUserial485
import time
import sys

# Open PRUserial485
PRUserial485.PRUserial485_open(6, b'S')

# Wait a little bit
time.sleep(2)

# Start sniffing
PRUserial485.PRUserial485_sniffer_start("/mnt/sniffer-log", 10*1024*1024, 4000*1024*1024)
print("Sniffer active. Python is entering a deep sleep loop...")
sys.stdout.flush()

# DEBUG-ONLY: MAX3107 RevID, read+stored unconditionally at PRU boot
# (PRUserial485.p:205-209, SPI command 0x1f), before Master/Slave dispatch
# even happens. Per the MAX3107 datasheet, a healthy chip reports 0xa1.
# If this is wrong, SPI communication between the PRU and the chip
# itself is broken on this board -- a hardware fault independent of
# anything downstream (bus wiring, traffic, etc.), since this read
# happens before the receive loop's WAIT_RECEIVED_SLAVE state is ever
# reached. Confirmed 0xa1 on BBB-NEW-2026: SPI/chip communication is
# healthy there, so a stuck WAIT_RECEIVED_SLAVE points further upstream
# (the bus connection itself), not at this board's hardware.
revid = PRUserial485.PRUserial485_shram(0)
print("MAX3107 RevID (shram offset 0): {:#04x} (expect 0xa1)".format(revid))
sys.stdout.flush()

# DEBUG-ONLY: trace the Slave-mode receive loop's checkpoint (offset 94,
# see DEBUG_STATE_* in PRUserial485.p) to localize a sniffer stall. Must
# poll from this same process: it's the one with an already-open PRU
# session (prudata mmap'd by the PRUserial485_open() call above), and a
# second process calling PRUserial485_open() itself to get its own
# mapping would reload PRU firmware and disrupt this session. Remove
# once the stall is found; replace with the plain sleep loop below.
DEBUG_STATE_NAMES = {
    1: "START_SLAVE (loop top)",
    2: "WAIT_RECEIVED_SLAVE (idle, waiting for first byte)",
    3: "GOT_FIRST_BYTE_SLAVE (configuring RxTimeout IRQ)",
    4: "RXLEVEL_AND_TIMEOUT_SLAVE (mid-message)",
    5: "STORE_LEFTBYTES_SLAVE (finalizing message)",
    6: "DATA_READY_SLAVE (signaling ARM)",
    7: "SEND_DATA_SLAVE (UNEXPECTED: something called write() on this connection)",
}

last_state = None
last_change_ts = time.monotonic()
while True:
    state = PRUserial485.PRUserial485_shram(94)
    now = time.monotonic()
    if state != last_state:
        print("[{:.1f}] state -> {} ({})".format(
            now, state, DEBUG_STATE_NAMES.get(state, "unknown")))
        sys.stdout.flush()
        last_state = state
        last_change_ts = now
    elif now - last_change_ts > 30:
        # Heartbeat: same state for 30+ seconds straight is the stall signal.
        print("[{:.1f}] still stuck at state {} ({}), {:.0f}s and counting".format(
            now, state, DEBUG_STATE_NAMES.get(state, "unknown"), now - last_change_ts))
        sys.stdout.flush()
        last_change_ts = now  # only re-print every 30s, not every poll
    time.sleep(1)

# Original idle loop, restore once the debug trace above is removed:
# while True:
#     time.sleep(86400)
