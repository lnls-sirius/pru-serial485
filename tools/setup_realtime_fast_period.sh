#!/usr/bin/env bash
# Experimental variant of setup_realtime.sh: same RT cgroup bandwidth chain,
# but with the period shrunk from the kernel's default 1s down to 10ms for
# the user.slice branch (user.slice, user-0.slice, the login session scope).
#
# Why: the default 1s period means a throttling event's worst-case blackout
# is period*(1-fraction), which at 1s/95% is up to 50ms, and at 1s/99% is
# still up to 10ms. The sniffer's on-chip byte-ring only holds ~4KB, which
# fills in well under 10ms at realistic bus speeds, so any blackout anywhere
# near that size overruns it regardless of how close to 100% the fraction is.
# Shrinking the period to 10ms bounds the worst-case blackout to ~1ms
# instead, comfortably under the ring's capacity, while keeping the same
# total RT bandwidth fraction the user.slice chain already had.
#
# Deliberately leaves root (/proc/sys/kernel/sched_rt_*) and system.slice
# completely untouched, unlike an earlier version of this script. The
# kernel's cgroup RT bandwidth check normalizes by rate (runtime/period),
# not raw microseconds, so user.slice can run its own 10ms/9ms (90%) pair
# while root stays at its original 1s/950000 (95%) and system.slice at its
# original 1s/50000 (5%): 5% + 90% = 95%, exactly what root already allows,
# so root's own bandwidth never needs to change. This also keeps every
# other cgroup on the system, and the kernel's own safety margin against a
# truly runaway RT thread, exactly as they were before this script ran.
#
# Must set BOTH cpu.rt_period_us and cpu.rt_runtime_us at every level in
# this branch: each cgroup enforces its own period/runtime pair
# independently, so leaving an ancestor at the default 1s period would
# still let that ancestor's own 1s-window accounting throttle everything
# beneath it for up to its own blackout duration, undoing the fix at the
# leaf.
#
# Must be rerun after every reboot and again in every new SSH session, same
# as setup_realtime.sh (this does everything that script does for the
# user.slice chain, plus the period change). Run as root, in the exact
# terminal that will launch python-sirius.
set -u

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root" >&2
    exit 1
fi

CGROUP_CPU=/sys/fs/cgroup/cpu
PERIOD=10000
USER_CHAIN_RUNTIME=9000
SHRINK_PLACEHOLDER=100

set_val() {
    local path="$1" value="$2"
    if [ ! -e "$path" ]; then
        echo "FAILED: $path does not exist" >&2
        return 1
    fi
    echo "$value" > "$path"
    local got
    got=$(cat "$path")
    if [ "$got" != "$value" ]; then
        echo "FAILED: $path is $got after writing $value" >&2
        return 1
    fi
    echo "OK: $path = $got"
}

session_scope=$(grep -o 'session-[0-9]*\.scope' /proc/self/cgroup | head -n1)
if [ -z "$session_scope" ]; then
    echo "FAILED: could not find session-N.scope in /proc/self/cgroup" \
         "(not running inside a login session cgroup?)" >&2
    exit 1
fi
echo "== current session: $session_scope =="

USER_SLICE="$CGROUP_CPU/user.slice"
USER_0_SLICE="$USER_SLICE/user-0.slice"
SESSION_SCOPE="$USER_0_SLICE/$session_scope"

echo "== phase 1: shrink this branch's runtimes to a tiny placeholder at the current (1s) period =="
echo "== (leaf to root within the branch: a parent can't shrink below what its own child still holds) =="
echo "== root and system.slice are never touched =="
set_val "$SESSION_SCOPE/cpu.rt_runtime_us" "$SHRINK_PLACEHOLDER" || exit 1
set_val "$USER_0_SLICE/cpu.rt_runtime_us" "$SHRINK_PLACEHOLDER" || exit 1
set_val "$USER_SLICE/cpu.rt_runtime_us" "$SHRINK_PLACEHOLDER" || exit 1

echo "== phase 2: shrink the period to ${PERIOD}us across this branch =="
echo "== (safe now: every runtime above is tiny, so it fits under any period) =="
set_val "$USER_SLICE/cpu.rt_period_us" "$PERIOD" || exit 1
set_val "$USER_0_SLICE/cpu.rt_period_us" "$PERIOD" || exit 1
set_val "$SESSION_SCOPE/cpu.rt_period_us" "$PERIOD" || exit 1

echo "== phase 3: raise runtimes back up to their final fraction, root of the branch down to leaf =="
set_val "$USER_SLICE/cpu.rt_runtime_us" "$USER_CHAIN_RUNTIME" || exit 1
set_val "$USER_0_SLICE/cpu.rt_runtime_us" "$USER_CHAIN_RUNTIME" || exit 1
set_val "$SESSION_SCOPE/cpu.rt_runtime_us" "$USER_CHAIN_RUNTIME" || exit 1

echo ""
echo "all RT cgroup values set for $session_scope, period=${PERIOD}us"
echo "worst-case contiguous RT blackout for this session's threads: ~$((PERIOD - USER_CHAIN_RUNTIME))us"
echo "now launch python-sirius in THIS terminal"
