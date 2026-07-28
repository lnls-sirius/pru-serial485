/*
sniffer.c


--------------------------------------------------------------------------------
Passive RS-485 traffic logger, built on top of PRUserial485 Slave mode
--------------------------------------------------------------------------------

Brazilian Synchrotron Light Laboratory (LNLS/CNPEM)
Controls Group | Electronic Instrumentation Group

Owns exactly two public entry points: PRUserial485_sniffer_start() and
PRUserial485_sniffer_stop(). Everything else in this file is internal.

This module assumes PRUserial485_open(baudrate, 'S') has already been
called (Slave mode, unmodified) and never touches that connection's
lifecycle itself: starting/stopping the sniffer's own logging and
owning the PRU connection are kept as separate concerns. In particular,
this file never calls close_PRU() and never sends anything on the bus.
*/

#include "PRUserial485.h"

// Hard compile-time guarantee: nothing in this file can transmit onto the
// bus, even by accident. This pragma is scoped to this translation unit
// only; it does not affect PRUserial485.c, which still legitimately
// needs send_data_PRU() for Master mode.
#pragma GCC poison send_data_PRU

#include "shram_mapping.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Upper bound on bytes pulled from recv_data_PRU() in a single batch:
// matches PRUserial485.c's own INCOMING_BUFF_SIZE, the real capacity of
// the host-side ring recv_data_PRU() reads from.
#define SNIFFER_MAX_BATCH_BYTES    100000

// Bound on how long the capture thread waits for monitorRecvBuffer's
// signal before re-checking sniffer_running; keeps shutdown responsive
// even if no more traffic ever arrives. Not a polling interval: while
// traffic is flowing, the thread wakes immediately on each signal.
// 100 ms
#define SNIFFER_WAIT_TIMEOUT_NS    100000000L

// Each event type gets its own magic byte, so a reader never has to
// decode a reason out of the count field: classification stays correct
// no matter how large a count value is.
#define SNIFFER_LOG_MAGIC_MESSAGE           0xA5
#define SNIFFER_LOG_MAGIC_OVERFLOW_ENTRIES  0xEE
#define SNIFFER_LOG_MAGIC_OVERFLOW_BYTES    0xEF
#define SNIFFER_LOG_MAGIC_RESYNC            0xED
#define SNIFFER_LOG_VERSION                 0x01
#define SNIFFER_LOG_HEADER_BYTES            18

// fsync() is a synchronous storage barrier. On SD/eMMC it can
// occasionally stall for tens of milliseconds (wear-leveling, GC
// pauses). Calling it per record let a single slow fsync stall the
// capture thread long enough for the 100KB host-side ring to wrap and
// overwrite unread data, i.e. the durability mechanism was itself
// causing the loss it was meant to guard against. write() alone is
// already safe against a process crash (kill -9, segfault), since the
// page cache survives that; only a kernel panic or power loss needs
// fsync, so batching it trades a small, bounded exposure to *those*
// specific failure modes for dramatically better throughput.
#define SNIFFER_FSYNC_EVERY_N_RECORDS  32


// --- Sniffer thread state ---
static volatile uint8_t sniffer_running = 0;
static volatile uint8_t sniffer_write_failed = 0;
static pthread_t sniffer_thread_id;

static int    sniffer_log_fd = -1;
static char   sniffer_log_dir[512];

// basename only, e.g. "sniff_...log"
static char   sniffer_log_current_name[256];

// bytes in the *current* file (for rotation)
static size_t sniffer_log_bytes_written = 0;
static size_t sniffer_rotate_bytes = 0;

// 0 = unlimited; total across all rotated files
static size_t sniffer_max_total_bytes = 0;
static uint32_t sniffer_seq = 0;
static unsigned sniffer_log_records_since_fsync = 0;


// The length-fifo ring lives on-chip (see shram_mapping.h), so reading
// an entry is just two ordinary shared-RAM byte reads through the
// library's existing public accessor: no DDR, no mmap, no cross-bus
// read at all. Safe to read without a stability retry (unlike the
// counter in PRUserial485.c): the PRU always finishes writing a ring
// slot before it publishes the counter value that makes that slot
// visible, so by the time this is called for a given ring_index, the
// PRU is done writing it.
static uint32_t sniffer_shram_read_length(uint32_t ring_index){
    uint16_t offset = SHRAM_OFFSET_LENFIFO_RING
                       + (uint16_t)((ring_index & (LENFIFO_DEPTH - 1)) * LENFIFO_ENTRY_BYTES);
    uint32_t value = 0;
    int b;
    for(b = 0; b < LENFIFO_ENTRY_BYTES; b++){
        value |= ((uint32_t)read_shram(offset + b)) << (8 * b);
    }
    return value;
}


static void write_u32_le(uint8_t *buf, uint32_t v){
    buf[0] = (uint8_t)(v >> 0);
    buf[1] = (uint8_t)(v >> 8);
    buf[2] = (uint8_t)(v >> 16);
    buf[3] = (uint8_t)(v >> 24);
}


static void write_u64_le(uint8_t *buf, uint64_t v){
    int b;
    for(b = 0; b < 8; b++){
        buf[b] = (uint8_t)(v >> (8 * b));
    }
}


// Deletes the oldest "sniff_*.log" files in sniffer_log_dir (filename
// order matches chronological order, since the timestamp fields are
// fixed-width for the foreseeable future) until the directory's total
// size fits within sniffer_max_total_bytes. Never deletes
// sniffer_log_current_name (the file actively being written to). Best
// effort: if the directory can't be scanned, does nothing and leaves
// ENOSPC/write-failure handling as the backstop.
static void sniffer_log_make_room(void){
    if(sniffer_max_total_bytes == 0){
        return;
    }

    for(;;){
        DIR *dir;
        struct dirent *entry;
        char oldest[256];
        char path[800];
        struct stat st;
        size_t total = 0;
        int have_oldest = 0;

        dir = opendir(sniffer_log_dir);
        if(dir == NULL){
            return;
        }

        oldest[0] = '\0';
        while((entry = readdir(dir)) != NULL){
            if(strncmp(entry->d_name, "sniff_", 6) != 0){
                continue;
            }
            snprintf(path, sizeof(path), "%s/%s", sniffer_log_dir, entry->d_name);
            if(stat(path, &st) != 0){
                continue;
            }
            total += (size_t)st.st_size;

            if(strcmp(entry->d_name, sniffer_log_current_name) == 0){
                // never a deletion candidate while active
                continue;
            }
            if(!have_oldest || strcmp(entry->d_name, oldest) < 0){
                strncpy(oldest, entry->d_name, sizeof(oldest) - 1);
                oldest[sizeof(oldest) - 1] = '\0';
                have_oldest = 1;
            }
        }
        closedir(dir);

        if(total <= sniffer_max_total_bytes){
            return;
        }
        if(!have_oldest){
            // nothing left we're allowed to delete
            return;
        }

        snprintf(path, sizeof(path), "%s/%s", sniffer_log_dir, oldest);
        fprintf(stderr, "[PRUserial485 sniffer] total log budget of %zu bytes exceeded, "
                        "deleting oldest log %s\n", sniffer_max_total_bytes, oldest);
        unlink(path);
    }
}


static int sniffer_log_open_new_file(void){
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
    if(fd < 0){
        return -1;
    }

    sniffer_log_fd = fd;
    sniffer_log_bytes_written = 0;
    sniffer_log_records_since_fsync = 0;
    return 0;
}


static void sniffer_log_write_raw(const uint8_t *buf, size_t len){
    ssize_t n;
    size_t written = 0;

    if(sniffer_log_fd < 0 || sniffer_write_failed){
        return;
    }

    while(written < len){
        n = write(sniffer_log_fd, buf + written, len - written);
        if(n < 0){
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

    if(sniffer_rotate_bytes > 0 && sniffer_log_bytes_written >= sniffer_rotate_bytes){
        // always flush fully before rotating away from this fd
        fsync(sniffer_log_fd);
        close(sniffer_log_fd);
        sniffer_log_fd = -1;
        sniffer_log_open_new_file();
    }
}


// Call once per logical record (not once per write(); a message record
// is header+payload, two calls to sniffer_log_write_raw()) to batch
// fsync() instead of paying its latency on every single one.
static void sniffer_log_maybe_fsync(void){
    sniffer_log_records_since_fsync++;
    if(sniffer_log_records_since_fsync >= SNIFFER_FSYNC_EVERY_N_RECORDS){
        if(sniffer_log_fd >= 0){
            fsync(sniffer_log_fd);
        }
        sniffer_log_records_since_fsync = 0;
    }
}


static void sniffer_log_write_message(const uint8_t *payload, uint32_t length){
    uint8_t header[SNIFFER_LOG_HEADER_BYTES];
    struct timespec ts;
    uint64_t ts_ns;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;

    header[0] = SNIFFER_LOG_MAGIC_MESSAGE;
    header[1] = SNIFFER_LOG_VERSION;
    write_u64_le(&header[2], ts_ns);
    write_u32_le(&header[10], sniffer_seq++);
    write_u32_le(&header[14], length);

    sniffer_log_write_raw(header, sizeof(header));
    if(length > 0){
        sniffer_log_write_raw(payload, length);
    }
    sniffer_log_maybe_fsync();
}


static void sniffer_log_write_event(uint8_t magic, uint32_t count){
    uint8_t header[SNIFFER_LOG_HEADER_BYTES];
    struct timespec ts;
    uint64_t ts_ns;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;

    header[0] = magic;
    header[1] = SNIFFER_LOG_VERSION;
    write_u64_le(&header[2], ts_ns);

    // ties event to the next real message's seq
    write_u32_le(&header[10], sniffer_seq);
    write_u32_le(&header[14], count);

    sniffer_log_write_raw(header, sizeof(header));
    sniffer_log_maybe_fsync();
}


static void sniffer_log_write_overflow(uint32_t lost_count, uint8_t magic, const char *reason){
    sniffer_log_write_event(magic, lost_count);
    fprintf(stderr, "[PRUserial485 sniffer] overflow: %u lost (%s)\n", lost_count, reason);
}


static void sniffer_log_write_resync(uint32_t delta){
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


static void *sniffer_capture_thread(void *arg){
    uint8_t *databuf;
    uint32_t entry_lengths[LENFIFO_DEPTH];
    uint32_t host_count;

    (void)arg;

    databuf = malloc(SNIFFER_MAX_BATCH_BYTES);
    if(databuf == NULL){
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

    while(sniffer_running && !sniffer_write_failed){
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
        if(deadline.tv_nsec >= 1000000000L){
            deadline.tv_sec += deadline.tv_nsec / 1000000000L;
            deadline.tv_nsec %= 1000000000L;
        }
        pthread_cond_timedwait(&sniffer_data_ready, &lock, &deadline);
        pru_count = sniffer_lenfifo_snapshot;
        pthread_mutex_unlock(&lock);

        if(!sniffer_running){
            break;
        }

        if(pru_count < host_count){
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
        if(new_count == 0){
            // timed out with nothing new yet, or spurious wakeup
            continue;
        }

        if(new_count > LENFIFO_DEPTH){
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
        for(k = 0; k < new_count; k++){
            uint32_t len = sniffer_shram_read_length(host_count + k);
            if(total_expected_bytes + len > SNIFFER_MAX_BATCH_BYTES){
                break;
            }
            entry_lengths[k] = len;
            total_expected_bytes += len;
        }

        // however many we decided to actually process this round
        new_count = k;

        if(new_count == 0){
            // A single message is larger than SNIFFER_MAX_BATCH_BYTES --
            // not possible given the byte-ring's real capacity, but avoid
            // spinning forever if it ever happens.
            continue;
        }

        got_bytes = 0;
        recv_data_PRU(databuf, &got_bytes, total_expected_bytes);

        if(got_bytes < total_expected_bytes){
            // Should not happen given the snapshot-synchronized signal
            // above; if it ever does, it is a genuine loss (e.g. the
            // on-chip byte-ring itself wrapped), not a timing artifact.
            sniffer_log_write_overflow(total_expected_bytes - got_bytes, SNIFFER_LOG_MAGIC_OVERFLOW_BYTES,
                                        "byte-ring yielded fewer bytes than length-fifo expected");
        }

        offset = 0;
        for(k = 0; k < new_count; k++){
            if(offset + entry_lengths[k] > got_bytes){
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


int PRUserial485_sniffer_start(const char *log_dir, size_t rotate_bytes, size_t max_total_bytes){
    if(sniffer_running){
        return ERR_SNIFFER_ALREADY_RUNNING;
    }

    strncpy(sniffer_log_dir, log_dir, sizeof(sniffer_log_dir) - 1);
    sniffer_log_dir[sizeof(sniffer_log_dir) - 1] = '\0';
    sniffer_rotate_bytes = rotate_bytes;
    sniffer_max_total_bytes = max_total_bytes;
    sniffer_log_current_name[0] = '\0';
    sniffer_seq = 0;
    sniffer_write_failed = 0;

    if(sniffer_log_open_new_file() != 0){
        return ERR_SNIFFER_LOG_OPEN;
    }

    sniffer_running = 1;
    if(pthread_create(&sniffer_thread_id, NULL, sniffer_capture_thread, NULL) != 0){
        sniffer_running = 0;
        close(sniffer_log_fd);
        sniffer_log_fd = -1;
        return ERR_SNIFFER_THREAD_CREATE;
    }

    return OK;
}


void PRUserial485_sniffer_stop(void){
    if(!sniffer_running){
        return;
    }

    sniffer_running = 0;
    pthread_join(sniffer_thread_id, NULL);

    if(sniffer_log_fd >= 0){
        // flush whatever's pending from a partial batch
        fsync(sniffer_log_fd);
        close(sniffer_log_fd);
        sniffer_log_fd = -1;
    }
}
