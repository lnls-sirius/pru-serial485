#!/usr/bin/env bash
# Set up the RT cgroup bandwidth chain the sniffer's SCHED_RR threads need
# to actually get real-time CPU time (requesting SCHED_RR alone is not
# enough; the kernel throttles RT threads to zero runtime in any cgroup
# whose cpu.rt_runtime_us is 0, silently, with no error at thread-create
# time). Must be rerun after every reboot (these values are not
# persistent) and again in every new SSH session (each session gets its
# own fresh session-N.scope, defaulting to 0, independent of reboot). See
# SNIFFER_SCHED_FIFO_VALIDATION.md for the full story.
#
# Run as root, in the exact terminal/session that will launch
# python-sirius.
set -u

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root" >&2
    exit 1
fi

CGROUP_CPU=/sys/fs/cgroup/cpu

set_rt() {
    local path="$1" value="$2"
    if [ ! -e "$path" ]; then
        echo "FAILED: $path does not exist" >&2
        return 1
    fi
    echo "$value" > "$path"
    local got
    got=$(cat "$path")
    if [ "$got" != "$value" ]; then
        echo "FAILED: $path is $got after writing $value (cgroup v1 requires" \
             "sum(children) <= parent at every level; a sibling is probably" \
             "holding too much budget)" >&2
        return 1
    fi
    echo "OK: $path = $got"
}

echo "== freeing system.slice's share first (raising user.slice needs room under the root's total budget) =="
set_rt "$CGROUP_CPU/system.slice/cpu.rt_runtime_us" 50000 || exit 1

echo "== raising user.slice chain, root to leaf =="
set_rt "$CGROUP_CPU/user.slice/cpu.rt_runtime_us" 900000 || exit 1
set_rt "$CGROUP_CPU/user.slice/user-0.slice/cpu.rt_runtime_us" 900000 || exit 1

session_scope=$(grep -o 'session-[0-9]*\.scope' /proc/self/cgroup | head -n1)
if [ -z "$session_scope" ]; then
    echo "FAILED: could not find session-N.scope in /proc/self/cgroup" \
         "(not running inside a login session cgroup?)" >&2
    exit 1
fi
echo "== current session: $session_scope =="
set_rt "$CGROUP_CPU/user.slice/user-0.slice/$session_scope/cpu.rt_runtime_us" 900000 || exit 1

echo "== rtprio rlimit for this shell (informational; comes from PAM, should already be 99) =="
ulimit -r

echo ""
echo "all RT cgroup values set for $session_scope"
echo "now launch python-sirius in THIS terminal"
