/*
sniffer.c


--------------------------------------------------------------------------------
Passive RS-485 traffic logger, built on top of PRUserial485 Passive mode
--------------------------------------------------------------------------------

Brazilian Synchrotron Light Laboratory (LNLS/CNPEM)
Controls Group | Electronic Instrumentation Group

Owns exactly two public entry points: PRUserial485_sniffer_start() and
PRUserial485_sniffer_stop(). Everything else in this file is internal.

This module assumes PRUserial485_open(baudrate, 'P') has already been
called (Passive mode) and never touches that connection's lifecycle
itself: starting/stopping the sniffer's own logging and owning the PRU
connection are kept as separate concerns. In particular, this file never
calls close_PRU().

No-transmit guarantee is structural, not a convention of this file:
Passive mode ('P') receives exactly like Slave mode ('S'), but the PRU
firmware itself never checks for outgoing data on a 'P' connection
(PRUserial485.p), and send_data_PRU() (PRUserial485.c) refuses
immediately if this connection is in Passive mode. That guarantee covers
every caller, not just this translation unit. Ordinary Slave mode, used
elsewhere for a real device replying to a master, is a distinct mode
value and is never affected by any of this.

Two-thread pipeline, by design: sniffer_capture_thread() drains the
PRU's on-chip byte-ring (only ~4KB) and never touches the log file or
storage in any way; it only ever formats a record and hands it to a
bounded in-memory queue (sniffer_queue_push_record()), dropping and
counting rather than blocking if that queue is full. sniffer_writer_
thread() is the only code that ever calls write()/fsync() on the log
fd. This means a slow disk (fsync() stalling tens of ms on FAT/SD/eMMC,
see SNIFFER_FSYNC_EVERY_N_RECORDS below) can now, at worst, cause a
bounded, visible queue-overflow drop (SNIFFER_LOG_MAGIC_QUEUE_OVERFLOW)
instead of starving PRU-ring draining and causing the much larger,
unbounded byte-ring-underrun losses that starvation used to cause.
*/

#include "PRUserial485.h"

#include "shram_mapping.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

// Upper bound on bytes pulled from recv_data_PRU() in a single batch:
// matches PRUserial485.c's own INCOMING_BUFF_SIZE, the real capacity of
// the host-side ring recv_data_PRU() reads from.
#define SNIFFER_MAX_BATCH_BYTES    100000

// A single message can't plausibly be larger than this: the on-chip
// receive ring in PRUserial485.c's monitorRecvBuffer (BUFF_RECV_STOP -
// BUFF_RECV_START, 0x2800 - 0x1803) is only 4093 bytes, the tightest
// buffer anywhere in this path, so nothing coherent could ever exceed
// it. A length-fifo entry above this is corrupt, not just large.
#define SNIFFER_MAX_PLAUSIBLE_MSG_LEN  4093

// Bound on how long the capture thread waits for monitorRecvBuffer's
// signal before re-checking sniffer_running; keeps shutdown responsive
// even if no more traffic ever arrives. Not a polling interval: while
// traffic is flowing, the thread wakes immediately on each signal.
// 100 ms
#define SNIFFER_WAIT_TIMEOUT_NS    100000000L

// The on-chip byte-ring the PRU drains into is small; if this thread
// gets starved under normal SCHED_OTHER contention (another process
// hogging the CPU, disk I/O from log rotation, etc.) for long enough,
// the ring wraps before it's read and messages are genuinely lost
// (see the byte-ring-underrun overflow path below). Real-time priority
// keeps this thread's scheduling independent of whatever else is
// running on the host. Modest by SCHED_RR standards (max is 99) so it
// still yields to anything with a genuine higher-priority realtime need.
//
// SCHED_RR, not SCHED_FIFO, and the same priority as PRUserial485.c's
// monitorRecvBuffer thread (MONITOR_THREAD_RT_PRIORITY), not lower: a
// strictly lower SCHED_FIFO priority here was tried and reverted after
// testing on real hardware, since it let monitorRecvBuffer starve this
// thread completely for as long as it had continuous traffic to
// process, losing tens of thousands of messages in one burst. SCHED_RR
// at equal priority is what actually fixes that: the kernel round-robins
// fairly between two threads at the same priority on its own. An
// explicit sched_yield() after every message on the monitorRecvBuffer
// side was also tried and reverted, since real traffic on this bus
// arrives in bursts of tens of thousands of messages/sec, and forcing a
// context switch on every single one at that rate cost far more than it
// saved (measured on real hardware: ~90% of traffic lost to periodic
// length-FIFO overflows, worse than not having this fix at all).
#define SNIFFER_THREAD_RT_PRIORITY  20

// Each event type gets its own magic byte, so a reader never has to
// decode a reason out of the count field: classification stays correct
// no matter how large a count value is.
#define SNIFFER_LOG_MAGIC_MESSAGE           0xA5
#define SNIFFER_LOG_MAGIC_OVERFLOW_ENTRIES  0xEE
#define SNIFFER_LOG_MAGIC_OVERFLOW_BYTES    0xEF
#define SNIFFER_LOG_MAGIC_RESYNC            0xED
#define SNIFFER_LOG_MAGIC_BAD_LENGTH        0xEC
#define SNIFFER_LOG_MAGIC_QUEUE_OVERFLOW    0xEB
#define SNIFFER_LOG_VERSION                 0x01
#define SNIFFER_LOG_HEADER_BYTES            18

// fsync() is a synchronous storage barrier that can stall tens of ms on
// SD/eMMC or FAT-formatted USB sticks. All fsync()s now happen on the
// writer thread (see sniffer_writer_thread below), never on the capture
// thread that drains the PRU's on-chip byte-ring, so a stall here can no
// longer starve that ring the way it once could. Still batched rather
// than done every record: write() alone already survives an ordinary
// process crash, only a kernel panic or power loss needs fsync, so
// batching it trades a small, bounded exposure to those for much better
// throughput. 256 was validated empirically in sniffer tests.
#define SNIFFER_FSYNC_EVERY_N_RECORDS  256

// Bytes buffered between the capture thread (producer) and the writer
// thread (consumer) that owns the log fd. Sized to comfortably absorb a
// multi-ms to low-tens-of-ms fsync() stall at realistic bus data rates
// without the capture thread ever having to wait for the writer: see
// sniffer_queue_push_record's drop-on-full policy below. 8 MiB is
// trivial memory on this hardware next to what it buys in stall
// tolerance.
#define SNIFFER_WRITER_QUEUE_BYTES  (8 * 1024 * 1024)

// Largest single record the capture thread can ever produce (header +
// the largest plausible message), so it can be built in a fixed-size
// local buffer before being handed to the queue.
#define SNIFFER_QUEUE_MAX_RECORD_BYTES  (SNIFFER_LOG_HEADER_BYTES + SNIFFER_MAX_PLAUSIBLE_MSG_LEN)


// --- Sniffer thread state ---
static volatile uint8_t sniffer_running = 0;
static volatile uint8_t sniffer_write_failed = 0;
static pthread_t sniffer_thread_id;

static int sniffer_log_fd = -1;
static char sniffer_log_dir[512];

// basename only, e.g. "sniff_...log"
static char   sniffer_log_current_name[256];

// bytes in the *current* file (for rotation)
static size_t sniffer_log_bytes_written = 0;
static size_t sniffer_rotate_bytes = 0;

// 0 = unlimited; total across all rotated files
static size_t sniffer_max_total_bytes = 0;

// Only ever written by the capture thread, but also read by the writer
// thread (sniffer_log_emit_queue_overflow) to annotate an event with the
// current position in the message sequence; volatile so that read is
// never cached stale across threads.
static volatile uint32_t sniffer_seq = 0;
static unsigned sniffer_log_records_since_fsync = 0;

// --- Capture-to-writer queue state ---
// Single-producer (capture thread), single-consumer (writer thread) byte
// ring of complete, pre-formatted log records, each stored as a 4-byte
// little-endian length prefix followed by that many record bytes. The
// producer never blocks on this: sniffer_queue_push_record() drops the
// whole record and counts it if there isn't room, rather than ever
// letting a slow writer/disk delay PRU-ring draining. See this file's
// top comment for the resulting guarantee.
static uint8_t  sniffer_queue_buf[SNIFFER_WRITER_QUEUE_BYTES];
static size_t   sniffer_queue_head = 0;
static size_t   sniffer_queue_tail = 0;
static size_t   sniffer_queue_used = 0;
static uint32_t sniffer_queue_dropped_records = 0;
static uint32_t sniffer_queue_dropped_bytes = 0;
static pthread_mutex_t sniffer_queue_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  sniffer_queue_not_empty = PTHREAD_COND_INITIALIZER;
static pthread_t       sniffer_writer_thread_id;
static volatile uint8_t sniffer_writer_running = 0;


// The length-FIFO ring lives on-chip (see shram_mapping.h), so reading
// an entry is just two ordinary shared-RAM byte reads through the
// library's existing public accessor. Safe to read without a stability
// retry (unlike the counter in PRUserial485.c): the PRU always finishes
// writing a ring slot before it publishes the counter value that makes
// that slot visible, so by the time this is called for a given ring_index,
// the PRU is done writing it.
static uint32_t sniffer_shram_read_length(uint32_t ring_index) {
    uint16_t offset = SHRAM_OFFSET_LENFIFO_RING
                       + (uint16_t)((ring_index & (LENFIFO_DEPTH - 1)) * LENFIFO_ENTRY_BYTES);
    uint32_t value = 0;
    int b;
    for(b = 0; b < LENFIFO_ENTRY_BYTES; b++) {
        value |= ((uint32_t)read_shram(offset + b)) << (8 * b);
    }
    return value;
}


static void write_u32_le(uint8_t *buf, uint32_t v) {
    buf[0] = (uint8_t)(v >> 0);
    buf[1] = (uint8_t)(v >> 8);
    buf[2] = (uint8_t)(v >> 16);
    buf[3] = (uint8_t)(v >> 24);
}


static void write_u64_le(uint8_t *buf, uint64_t v) {
    int b;
    for(b = 0; b < 8; b++) {
        buf[b] = (uint8_t)(v >> (8 * b));
    }
}


// Copies len bytes into the ring at sniffer_queue_tail, wrapping as
// needed, and advances tail/used. Caller must hold sniffer_queue_lock
// and must already have verified len bytes of room exist.
static void sniffer_queue_put_locked(const uint8_t *data, size_t len) {
    size_t first = SNIFFER_WRITER_QUEUE_BYTES - sniffer_queue_tail;
    if(first > len) {
        first = len;
    }
    memcpy(sniffer_queue_buf + sniffer_queue_tail, data, first);
    if(len > first) {
        memcpy(sniffer_queue_buf, data + first, len - first);
    }
    sniffer_queue_tail = (sniffer_queue_tail + len) % SNIFFER_WRITER_QUEUE_BYTES;
    sniffer_queue_used += len;
}


// Copies len bytes out of the ring from sniffer_queue_head, wrapping as
// needed, and advances head/used. Caller must hold sniffer_queue_lock
// and must already have verified len bytes are actually queued.
static void sniffer_queue_get_locked(uint8_t *out, size_t len) {
    size_t first = SNIFFER_WRITER_QUEUE_BYTES - sniffer_queue_head;
    if(first > len) {
        first = len;
    }
    memcpy(out, sniffer_queue_buf + sniffer_queue_head, first);
    if(len > first) {
        memcpy(out + first, sniffer_queue_buf, len - first);
    }
    sniffer_queue_head = (sniffer_queue_head + len) % SNIFFER_WRITER_QUEUE_BYTES;
    sniffer_queue_used -= len;
}


// Hands one already-formatted, complete log record to the writer thread.
// Never blocks: if the queue doesn't have room, the record is dropped
// and counted (sniffer_queue_dropped_records/_bytes) instead, so a slow
// writer/disk can never delay the caller. The writer thread reports
// accumulated drops as a SNIFFER_LOG_MAGIC_QUEUE_OVERFLOW event the next
// time it has room (sniffer_log_emit_queue_overflow). Returns nonzero if
// the record was actually queued.
static int sniffer_queue_push_record(const uint8_t *rec, uint32_t rec_len) {
    uint8_t len_prefix[4];
    size_t needed = 4 + (size_t)rec_len;
    int pushed;

    pthread_mutex_lock(&sniffer_queue_lock);
    if(sniffer_queue_used + needed > SNIFFER_WRITER_QUEUE_BYTES) {
        sniffer_queue_dropped_records++;
        sniffer_queue_dropped_bytes += rec_len;
        pushed = 0;
    } else {
        write_u32_le(len_prefix, rec_len);
        sniffer_queue_put_locked(len_prefix, sizeof(len_prefix));
        sniffer_queue_put_locked(rec, rec_len);
        pthread_cond_signal(&sniffer_queue_not_empty);
        pushed = 1;
    }
    pthread_mutex_unlock(&sniffer_queue_lock);
    return pushed;
}


// Deletes the oldest "sniff_*.log" files in sniffer_log_dir (filename
// order matches chronological order, since the timestamp fields are
// fixed-width for the foreseeable future) until the directory's total
// size fits within sniffer_max_total_bytes. Never deletes
// sniffer_log_current_name (the file actively being written to). Best
// effort: if the directory can't be scanned, does nothing and leaves
// ENOSPC/write-failure handling as the backstop.
static void sniffer_log_make_room(void) {
    if(sniffer_max_total_bytes == 0) {
        return;
    }

    for(;;) {
        DIR *dir;
        struct dirent *entry;
        char oldest[256];
        char path[800];
        struct stat st;
        size_t total = 0;
        int have_oldest = 0;

        dir = opendir(sniffer_log_dir);
        if(dir == NULL) {
            return;
        }

        oldest[0] = '\0';
        while((entry = readdir(dir)) != NULL) {
            if(strncmp(entry->d_name, "sniff_", 6) != 0) {
                continue;
            }
            snprintf(path, sizeof(path), "%s/%s", sniffer_log_dir, entry->d_name);
            if(stat(path, &st) != 0) {
                continue;
            }
            total += (size_t)st.st_size;

            if(strcmp(entry->d_name, sniffer_log_current_name) == 0) {
                // never a deletion candidate while active
                continue;
            }
            if(!have_oldest || strcmp(entry->d_name, oldest) < 0) {
                strncpy(oldest, entry->d_name, sizeof(oldest) - 1);
                oldest[sizeof(oldest) - 1] = '\0';
                have_oldest = 1;
            }
        }
        closedir(dir);

        if(total <= sniffer_max_total_bytes) {
            return;
        }
        if(!have_oldest) {
            // nothing left we're allowed to delete
            return;
        }

        snprintf(path, sizeof(path), "%s/%s", sniffer_log_dir, oldest);
        fprintf(stderr, "[PRUserial485 sniffer] total log budget of %zu bytes exceeded, "
                        "deleting oldest log %s\n", sniffer_max_total_bytes, oldest);
        unlink(path);
    }
}


static int sniffer_log_open_new_file(void) {
    char path[800];
    char name[256];
    struct timespec ts;
    int fd;

    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(name, sizeof(name), "sniff_%ld_%09ld.log", (long)ts.tv_sec, ts.tv_nsec);
    snprintf(path, sizeof(path), "%s/%s", sniffer_log_dir, name);

    // Updating this before make_room() means the *previous* current file
    // (if any) is no longer protected from deletion, which is correct,
    // since it's now just another closed, historical file like any other.
    strncpy(sniffer_log_current_name, name, sizeof(sniffer_log_current_name) - 1);
    sniffer_log_current_name[sizeof(sniffer_log_current_name) - 1] = '\0';

    sniffer_log_make_room();

    fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if(fd < 0) {
        return -1;
    }

    sniffer_log_fd = fd;
    sniffer_log_bytes_written = 0;
    sniffer_log_records_since_fsync = 0;
    return 0;
}


static void sniffer_log_write_raw(const uint8_t *buf, size_t len) {
    ssize_t n;
    size_t written = 0;

    if(sniffer_log_fd < 0 || sniffer_write_failed) {
        return;
    }

    while(written < len) {
        n = write(sniffer_log_fd, buf + written, len - written);
        if(n < 0){
            // Catch an interrupted system call
            if(errno == EINTR){
                continue;
            }
            // Can't write an error into the log if the log itself is what's
            // failing (disk full, I/O error, ...), so report loudly to
            // stderr and stop the capture thread rather than silently
            // continuing to "log" nothing, which would be exactly the
            // silent-loss failure mode this tool exists to prevent.
            fprintf(stderr,
                    "[PRUserial485 sniffer] FATAL: write() failed (%s), "
                    "stopping capture, log can no longer be trusted complete\n",
                    strerror(errno));
            sniffer_write_failed = 1;
            return;
        }
        written += (size_t)n;
    }

    sniffer_log_bytes_written += len;
}


// Call once per logical record (not once per write(); a message record
// is header+payload, two calls to sniffer_log_write_raw()) to batch
// fsync() instead of paying its latency on every single one.
static void sniffer_log_maybe_fsync(void) {
    sniffer_log_records_since_fsync++;
    if(sniffer_log_records_since_fsync >= SNIFFER_FSYNC_EVERY_N_RECORDS) {
        if(sniffer_log_fd >= 0) {
            fsync(sniffer_log_fd);
        }
        sniffer_log_records_since_fsync = 0;
    }
}


// Call once per logical record, after every sniffer_log_write_raw() call
// for it has completed, never in the middle of one. A record's header
// and payload are written as two separate sniffer_log_write_raw() calls;
// rotating between them would strand the header (with no payload) at the
// end of the old file and dump the orphaned payload, unprefixed, at the
// start of the new one, breaking log framing without losing any actual
// data. Checking only here, at record boundaries, keeps that impossible.
static void sniffer_log_maybe_rotate(void) {
    if(sniffer_rotate_bytes > 0 && sniffer_log_bytes_written >= sniffer_rotate_bytes) {
        // always flush fully before rotating away from this fd
        fsync(sniffer_log_fd);
        close(sniffer_log_fd);
        sniffer_log_fd = -1;
        sniffer_log_open_new_file();
    }
}


// Builds the record locally and hands it to the writer queue rather than
// writing it directly (see this file's top comment): sniffer_seq is only
// actually consumed (incremented) if the record is accepted by the
// queue, so a message dropped here for lack of queue room never leaves
// a gap in the seq values that DO make it into the log - it simply never
// allocates one. The drop itself is still visible, via the aggregate
// byte/record counts sniffer_log_emit_queue_overflow() reports.
static void sniffer_log_write_message(const uint8_t *payload, uint32_t length) {
    uint8_t rec[SNIFFER_QUEUE_MAX_RECORD_BYTES];
    struct timespec ts;
    uint64_t ts_ns;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;

    rec[0] = SNIFFER_LOG_MAGIC_MESSAGE;
    rec[1] = SNIFFER_LOG_VERSION;
    write_u64_le(&rec[2], ts_ns);
    write_u32_le(&rec[10], sniffer_seq);
    write_u32_le(&rec[14], length);
    if(length > 0) {
        memcpy(rec + SNIFFER_LOG_HEADER_BYTES, payload, length);
    }

    if(sniffer_queue_push_record(rec, SNIFFER_LOG_HEADER_BYTES + length)) {
        sniffer_seq++;
    }
}


// Builds an event record and hands it to the writer queue. count uses
// the current, not-yet-consumed sniffer_seq (ties the event to the next
// real message's seq) - this never touches sniffer_seq itself, so an
// event dropped for lack of queue room is folded into the same
// aggregate drop counters as a dropped message.
static void sniffer_log_write_event(uint8_t magic, uint32_t count) {
    uint8_t rec[SNIFFER_LOG_HEADER_BYTES];
    struct timespec ts;
    uint64_t ts_ns;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;

    rec[0] = magic;
    rec[1] = SNIFFER_LOG_VERSION;
    write_u64_le(&rec[2], ts_ns);
    write_u32_le(&rec[10], sniffer_seq);
    write_u32_le(&rec[14], count);

    sniffer_queue_push_record(rec, sizeof(rec));
}


// Writer-thread-only: reports accumulated writer-queue drops as a log
// event, exactly like sniffer_log_write_message()/_event() do for their
// own record types, except this one is written directly (this thread
// already owns the fd and isn't going through its own queue) and its
// count is a byte count of dropped *record* bytes (message and/or event
// records alike), not payload bytes, since a drop here can hit either.
static void sniffer_log_emit_queue_overflow(uint32_t dropped_records, uint32_t dropped_bytes) {
    uint8_t rec[SNIFFER_LOG_HEADER_BYTES];
    struct timespec ts;
    uint64_t ts_ns;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;

    rec[0] = SNIFFER_LOG_MAGIC_QUEUE_OVERFLOW;
    rec[1] = SNIFFER_LOG_VERSION;
    write_u64_le(&rec[2], ts_ns);
    write_u32_le(&rec[10], sniffer_seq);
    write_u32_le(&rec[14], dropped_bytes);

    sniffer_log_write_raw(rec, sizeof(rec));
    sniffer_log_maybe_fsync();
    sniffer_log_maybe_rotate();

    fprintf(stderr,
            "[PRUserial485 sniffer] writer queue overflow: %u record(s) / %u byte(s) "
            "dropped before reaching disk (writer thread could not keep up)\n",
            dropped_records, dropped_bytes);
}


static void sniffer_log_write_overflow(uint32_t lost_count, uint8_t magic, const char *reason) {
    sniffer_log_write_event(magic, lost_count);
    fprintf(stderr, "[PRUserial485 sniffer] overflow: %u lost (%s)\n", lost_count, reason);
}


static void sniffer_log_write_bad_length(uint32_t bad_len) {
    // A length-fifo entry this large cannot be a real message on this
    // bus (see SNIFFER_MAX_PLAUSIBLE_MSG_LEN): a logic error somewhere
    // upstream, not expected in normal operation. Trusting it would
    // misalign every subsequent slice offset into databuf for the rest
    // of this batch, silently writing garbage into the log instead of
    // real messages. Discard the rest of this batch instead; the
    // remaining entries are simply picked up on the next iteration.
    sniffer_log_write_event(SNIFFER_LOG_MAGIC_BAD_LENGTH, bad_len);
    fprintf(stderr,
            "[PRUserial485 sniffer] implausible length-fifo entry (%u bytes), "
            "discarding rest of this batch, exact loss (if any) unknown\n", bad_len);
}


static void sniffer_log_write_resync(uint32_t delta) {
    // We know host_count was `delta` messages ahead of pru_count, which
    // should not be possible during normal operation, but not whether
    // that means `delta` messages were actually lost, or the discrepancy
    // came from somewhere else entirely. Report the anomaly honestly
    // rather than guessing at a loss count we can't actually justify.
    sniffer_log_write_event(SNIFFER_LOG_MAGIC_RESYNC, delta);
    fprintf(stderr,
            "[PRUserial485 sniffer] baseline resync: host_count was %u ahead of "
            "pru_count, resynced, exact loss (if any) unknown\n", delta);
}


// Sole owner of sniffer_log_fd once running: pops one complete record at
// a time off the queue (a fast, lock-held memcpy only, no I/O under the
// lock) and only then does the actual write()/fsync()/rotate() work,
// with the lock released. This is what keeps a slow disk from ever
// being able to delay sniffer_capture_thread(): the two threads only
// ever interact through the brief, bounded queue operations above.
//
// Runs at default (non-realtime) scheduling, deliberately: unlike the
// capture thread, nothing here has a hard latency requirement anymore
// (that guarantee now lives in the queue's drop-on-full policy), so
// there is no reason for it to compete for the same limited real-time
// budget as the capture and monitorRecvBuffer threads.
static void *sniffer_writer_thread(void *arg) {
    uint8_t rec_buf[SNIFFER_QUEUE_MAX_RECORD_BYTES];

    (void)arg;

    for(;;) {
        uint32_t rec_len = 0;
        uint32_t dropped_records = 0;
        uint32_t dropped_bytes = 0;
        int have_record = 0;

        pthread_mutex_lock(&sniffer_queue_lock);
        while(sniffer_queue_used == 0 && sniffer_writer_running) {
            struct timespec deadline;
            clock_gettime(CLOCK_REALTIME, &deadline);
            deadline.tv_nsec += SNIFFER_WAIT_TIMEOUT_NS;
            if(deadline.tv_nsec >= 1000000000L) {
                deadline.tv_sec += deadline.tv_nsec / 1000000000L;
                deadline.tv_nsec %= 1000000000L;
            }
            pthread_cond_timedwait(&sniffer_queue_not_empty, &sniffer_queue_lock, &deadline);
        }

        dropped_records = sniffer_queue_dropped_records;
        dropped_bytes = sniffer_queue_dropped_bytes;
        sniffer_queue_dropped_records = 0;
        sniffer_queue_dropped_bytes = 0;

        if(sniffer_queue_used > 0) {
            uint8_t len_prefix[4];
            sniffer_queue_get_locked(len_prefix, sizeof(len_prefix));
            rec_len = (uint32_t)len_prefix[0] | ((uint32_t)len_prefix[1] << 8)
                      | ((uint32_t)len_prefix[2] << 16) | ((uint32_t)len_prefix[3] << 24);
            sniffer_queue_get_locked(rec_buf, rec_len);
            have_record = 1;
        }
        pthread_mutex_unlock(&sniffer_queue_lock);

        // Emitted after unlocking: this itself does file I/O, and must
        // never happen while other threads might be waiting on the lock
        // just to push/pop, not to touch the disk.
        if(dropped_records > 0) {
            sniffer_log_emit_queue_overflow(dropped_records, dropped_bytes);
        }
        if(have_record) {
            sniffer_log_write_raw(rec_buf, rec_len);
            sniffer_log_maybe_fsync();
            sniffer_log_maybe_rotate();
        }

        if(!have_record && !sniffer_writer_running) {
            // Queue confirmed empty and told to stop: fully drained.
            break;
        }
    }

    return NULL;
}


static void *sniffer_capture_thread(void *arg) {
    uint8_t *databuf;
    uint32_t entry_lengths[LENFIFO_DEPTH];
    uint32_t host_count;

    (void)arg;

    databuf = malloc(SNIFFER_MAX_BATCH_BYTES);
    if(databuf == NULL) {
        return NULL;
    }

    // Baseline: taken from sniffer_lenfifo_snapshot rather than a fresh
    // read_shram() call, so it can never appear to be "ahead of" the
    // first snapshot value this loop later compares against (which would
    // otherwise underflow the unsigned subtraction below into a bogus,
    // enormous "new_count" on the very first iteration). This means a
    // handful of messages that completed between PRUserial485_open() and
    // sniffer_start(), but that monitorRecvBuffer hadn't yet processed,
    // may still be captured; that is harmless and preferable to the
    // alternative.
    pthread_mutex_lock(&lock);
    host_count = sniffer_lenfifo_snapshot;
    pthread_mutex_unlock(&lock);

    while(sniffer_running && !sniffer_write_failed) {
        uint32_t pru_count;
        uint32_t new_count;
        uint32_t total_expected_bytes;
        uint32_t got_bytes;
        uint32_t k;
        size_t offset;
        struct timespec deadline;

        // Wait for monitorRecvBuffer to signal that it has actually
        // finished copying new bytes into its host-side ring.
        // sniffer_lenfifo_snapshot is updated under the same lock, at the
        // same instant as that copy, so by construction, the count read
        // below can never be ahead of what recv_data_PRU() can deliver.
        // A bounded wait (rather than pthread_cond_wait) just keeps this
        // loop responsive to sniffer_running flipping to 0 even if no
        // more traffic ever arrives; it is not a polling interval.
        pthread_mutex_lock(&lock);
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_nsec += SNIFFER_WAIT_TIMEOUT_NS;
        if(deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec += deadline.tv_nsec / 1000000000L;
            deadline.tv_nsec %= 1000000000L;
        }
        pthread_cond_timedwait(&sniffer_data_ready, &lock, &deadline);
        pru_count = sniffer_lenfifo_snapshot;
        pthread_mutex_unlock(&lock);

        if(!sniffer_running) {
            break;
        }

        if(pru_count < host_count) {
            // Should not happen: host_count and pru_count are both
            // sourced from the same monotonic counter, always read under
            // `lock`, but guard against it rather than let this
            // subtraction wrap into a bogus multi-billion "loss" that
            // then cascades into the overflow-handling branch below
            // (whose own arithmetic assumes new_count is a real,
            // non-wrapped value). Resync visibly rather than silently.
            sniffer_log_write_resync(host_count - pru_count);
            host_count = pru_count;
            continue;
        }

        new_count = pru_count - host_count;
        if(new_count == 0) {
            // timed out with nothing new yet, or spurious wakeup
            continue;
        }

        if(new_count > LENFIFO_DEPTH) {
            uint32_t lost_entries = new_count - LENFIFO_DEPTH;
            sniffer_log_write_overflow(lost_entries, SNIFFER_LOG_MAGIC_OVERFLOW_ENTRIES,
                                        "length-fifo entries overwritten before being read");
            // Safe: new_count > LENFIFO_DEPTH here implies
            // pru_count = host_count + new_count > host_count + LENFIFO_DEPTH,
            // so pru_count - LENFIFO_DEPTH cannot underflow.
            host_count = pru_count - LENFIFO_DEPTH;
            new_count = LENFIFO_DEPTH;
        }
        // No further clamp needed here: new_count <= LENFIFO_DEPTH now,
        // which is exactly entry_lengths[]'s size.

        total_expected_bytes = 0;
        for(k = 0; k < new_count; k++) {
            uint32_t len = sniffer_shram_read_length(host_count + k);
            // Checked before the running total below, not after: a
            // sufficiently large garbage len could otherwise overflow
            // that addition and wrap back under the budget, defeating
            // this check entirely.
            if(len > SNIFFER_MAX_PLAUSIBLE_MSG_LEN) {
                sniffer_log_write_bad_length(len);
                break;
            }
            if(total_expected_bytes + len > SNIFFER_MAX_BATCH_BYTES) {
                break;
            }
            entry_lengths[k] = len;
            total_expected_bytes += len;
        }

        // however many we decided to actually process this round
        new_count = k;

        if(new_count == 0) {
            // Either a single message is larger than SNIFFER_MAX_BATCH_BYTES
            // (not possible given the byte-ring's real capacity) or the
            // very first entry this round had an implausible length
            // (already reported above). Either way, avoid spinning
            // forever if it ever happens.
            continue;
        }

        got_bytes = 0;
        recv_data_PRU(databuf, &got_bytes, total_expected_bytes);

        if(got_bytes < total_expected_bytes) {
            // Should not happen given the snapshot-synchronized signal
            // above; if it ever does, it is a genuine loss (e.g. the
            // on-chip byte-ring itself wrapped), not a timing artifact.
            sniffer_log_write_overflow(total_expected_bytes - got_bytes,
                                       SNIFFER_LOG_MAGIC_OVERFLOW_BYTES,
                                       "byte-ring yielded fewer bytes than length-fifo expected");
        }

        offset = 0;
        for(k = 0; k < new_count; k++) {
            if(offset + entry_lengths[k] > got_bytes) {
                // ran out of real bytes for this message
                break;
            }
            sniffer_log_write_message(databuf + offset, entry_lengths[k]);
            offset += entry_lengths[k];
        }

        host_count += new_count;
    }

    free(databuf);
    return NULL;
}


int PRUserial485_sniffer_start(const char *log_dir, size_t rotate_bytes, size_t max_total_bytes) {
    // Must be checked before anything else: running the sniffer on a
    // Master or plain Slave connection would apply Passive-mode
    // assumptions (see this file's top comment) to a connection that
    // doesn't have Passive mode's firmware guarantees backing them.
    if(read_shram(SHRAM_OFFSET_485_MODE) != 'P') {
        return ERR_SNIFFER_NOT_PASSIVE_MODE;
    }

    if(sniffer_running) {
        return ERR_SNIFFER_ALREADY_RUNNING;
    }

    strncpy(sniffer_log_dir, log_dir, sizeof(sniffer_log_dir) - 1);
    sniffer_log_dir[sizeof(sniffer_log_dir) - 1] = '\0';
    sniffer_rotate_bytes = rotate_bytes;
    sniffer_max_total_bytes = max_total_bytes;
    sniffer_log_current_name[0] = '\0';
    sniffer_seq = 0;
    sniffer_write_failed = 0;

    // Fresh queue for this session: a previous session always drains to
    // empty on stop (see PRUserial485_sniffer_stop()), but reset
    // explicitly rather than relying on that.
    sniffer_queue_head = 0;
    sniffer_queue_tail = 0;
    sniffer_queue_used = 0;
    sniffer_queue_dropped_records = 0;
    sniffer_queue_dropped_bytes = 0;

    if(sniffer_log_open_new_file() != 0) {
        return ERR_SNIFFER_LOG_OPEN;
    }

    // Start the writer before the capture thread, so the queue always
    // has a consumer from the moment traffic can start arriving.
    sniffer_writer_running = 1;
    if(pthread_create(&sniffer_writer_thread_id, NULL, sniffer_writer_thread, NULL) != 0) {
        sniffer_writer_running = 0;
        close(sniffer_log_fd);
        sniffer_log_fd = -1;
        return ERR_SNIFFER_THREAD_CREATE;
    }

    sniffer_running = 1;

    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    param.sched_priority = SNIFFER_THREAD_RT_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);

    if(pthread_create(&sniffer_thread_id, &attr, sniffer_capture_thread, NULL) != 0) {
        // Most likely cause: no CAP_SYS_NICE (not running as root).
        // Fall back to default scheduling rather than refusing to
        // capture at all, since losing the priority boost is better than
        // losing the sniffer.
        fprintf(stderr, "[PRUserial485 sniffer] could not start capture thread with "
                "SCHED_RR priority %d (missing CAP_SYS_NICE?), falling back to "
                "default scheduling\n", SNIFFER_THREAD_RT_PRIORITY);
        if(pthread_create(&sniffer_thread_id, NULL, sniffer_capture_thread, NULL) != 0) {
            pthread_attr_destroy(&attr);
            sniffer_running = 0;

            sniffer_writer_running = 0;
            pthread_mutex_lock(&sniffer_queue_lock);
            pthread_cond_broadcast(&sniffer_queue_not_empty);
            pthread_mutex_unlock(&sniffer_queue_lock);
            pthread_join(sniffer_writer_thread_id, NULL);

            close(sniffer_log_fd);
            sniffer_log_fd = -1;
            return ERR_SNIFFER_THREAD_CREATE;
        }
    }

    pthread_attr_destroy(&attr);
    return OK;
}


void PRUserial485_sniffer_stop(void) {
    if(!sniffer_running) {
        return;
    }

    sniffer_running = 0;
    pthread_join(sniffer_thread_id, NULL);

    // Capture thread is done producing; let the writer thread drain
    // whatever's still queued before stopping it too, so a clean stop
    // never loses anything that was already accepted into the queue.
    sniffer_writer_running = 0;
    pthread_mutex_lock(&sniffer_queue_lock);
    pthread_cond_broadcast(&sniffer_queue_not_empty);
    pthread_mutex_unlock(&sniffer_queue_lock);
    pthread_join(sniffer_writer_thread_id, NULL);

    if(sniffer_log_fd >= 0) {
        // flush whatever's pending from a partial batch
        fsync(sniffer_log_fd);
        close(sniffer_log_fd);
        sniffer_log_fd = -1;
    }
}
