#!/bin/bash
#
# run_test.sh - Full MCTP integration test for pldm issue #70
#
# This script orchestrates the end-to-end firmware update test:
#   1. Launches QEMU with an extra serial port for MCTP
#   2. Waits for the BMC to boot
#   3. Configures MCTP inside the guest
#   4. Starts the PLDM device simulator on the host
#   5. Copies the firmware package into the guest
#   6. Monitors pldmd for the expected behavior
#
# Usage: sudo ./run_test.sh [--skip-qemu]
#
# Prerequisites:
#   - Built QEMU image at OPENBMC_BUILD_DIR
#   - mctp-serial kernel module available (modprobe mctp-serial)
#   - Host tools built (mctp CLI, libpldm, pldm_sim)
#

set -euo pipefail

# ===== Configuration =====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENBMC_BUILD_DIR="${OPENBMC_BUILD_DIR:-/home/gmbeihl/claude/openbmc-build/build/evb-ast2600}"
IMAGE_DIR="$OPENBMC_BUILD_DIR/tmp/deploy/images/evb-ast2600"
MTD_IMAGE="$IMAGE_DIR/obmc-phosphor-image-evb-ast2600.static.mtd"

PLDM_SIM="$SCRIPT_DIR/build-host/pldm_sim"
USER_HOME="/home/gmbeihl"
MCTP_TOOL="$USER_HOME/claude/mctp-host/build/mctp"
LIBPLDM_DIR="$USER_HOME/claude/openbmc/libpldm/build-host/src"
FW_PKG="$SCRIPT_DIR/test_fw_pkg.pldm"

SSH_PORT=2222
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5 -o LogLevel=ERROR"
SSH_CMD="ssh $SSH_OPTS -p $SSH_PORT root@localhost"
SCP_CMD="scp $SSH_OPTS -P $SSH_PORT"

# Guest MCTP serial device (UART3 → ttyS2 with our QEMU serial mapping)
GUEST_SERIAL="/dev/ttyS2"
# MCTP EIDs
BMC_EID=8
FD_EID=20

SKIP_QEMU=false
QEMU_PID=""
PLDM_SIM_PID=""

# ===== Color output =====
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${BLUE}[INFO]${NC} $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n${BLUE}======== $* ========${NC}"; }

# ===== Cleanup =====
cleanup() {
    log_step "Cleanup"
    if [[ -n "$PLDM_SIM_PID" ]]; then
        log_info "Stopping pldm-sim (PID $PLDM_SIM_PID)"
        kill "$PLDM_SIM_PID" 2>/dev/null || true
        wait "$PLDM_SIM_PID" 2>/dev/null || true
    fi
    if [[ -n "$QEMU_PID" ]] && [[ "$SKIP_QEMU" == "false" ]]; then
        log_info "Stopping QEMU (PID $QEMU_PID)"
        kill "$QEMU_PID" 2>/dev/null || true
        wait "$QEMU_PID" 2>/dev/null || true
    fi
    log_info "Cleanup complete"
}
trap cleanup EXIT

# ===== Parse arguments =====
for arg in "$@"; do
    case "$arg" in
        --skip-qemu) SKIP_QEMU=true ;;
        --help|-h)
            echo "Usage: $0 [--skip-qemu]"
            echo ""
            echo "  --skip-qemu  Skip QEMU launch (assume it's already running)"
            exit 0
            ;;
    esac
done

# ===== Preflight checks =====
log_step "Preflight Checks"

# Check we're root (needed for MCTP serial line discipline)
if [[ $EUID -ne 0 ]]; then
    log_error "This script must be run as root (needed for mctp-serial line discipline)"
    exit 1
fi

# Check required files exist
for f in "$PLDM_SIM" "$MCTP_TOOL" "$FW_PKG"; do
    if [[ ! -f "$f" ]]; then
        log_error "Missing: $f"
        exit 1
    fi
done

if [[ "$SKIP_QEMU" == "false" ]]; then
    if [[ ! -f "$MTD_IMAGE" ]]; then
        log_error "Missing QEMU image: $MTD_IMAGE"
        exit 1
    fi
fi

# Check mctp-serial module
if ! modprobe -n mctp-serial 2>/dev/null; then
    log_warn "mctp-serial module may not be available"
fi

log_ok "Preflight checks passed"

# ===== Phase 1: Launch QEMU =====
log_step "Phase 1: QEMU Launch"

if [[ "$SKIP_QEMU" == "true" ]]; then
    log_info "Skipping QEMU launch (--skip-qemu)"
    log_info "Ensure QEMU is running with an extra -serial pty for MCTP"
    read -p "Enter the PTY path for the MCTP serial (e.g., /dev/pts/3): " PTY_PATH
else
    # Create a working copy of the MTD image so we don't modify the original
    WORK_MTD="/tmp/test-openbmc.mtd"
    log_info "Copying MTD image to $WORK_MTD"
    cp "$MTD_IMAGE" "$WORK_MTD"

    log_info "Launching QEMU..."
    # Serial mapping for ast2600-evb:
    #   -serial mon:stdio  → UART5 (ttyS4, console)
    #   -serial null       → UART1 (ttyS0)
    #   -serial null       → UART2 (ttyS1)
    #   -serial pty        → UART3 (ttyS2) ← MCTP serial
    qemu-system-arm \
        -machine ast2600-evb \
        -m 1G \
        -drive file="$WORK_MTD",if=mtd,format=raw \
        -serial mon:stdio \
        -serial null \
        -serial null \
        -serial pty \
        -net nic -net user,hostfwd=tcp::${SSH_PORT}-:22 \
        -nographic \
        2>&1 | tee /tmp/qemu-output.log &
    QEMU_PID=$!
    log_info "QEMU PID: $QEMU_PID"

    # Wait for QEMU to print the PTY path
    log_info "Waiting for QEMU PTY assignment..."
    for i in $(seq 1 30); do
        if grep -q "char device redirected to /dev/pts/" /tmp/qemu-output.log 2>/dev/null; then
            break
        fi
        sleep 1
    done

    PTY_PATH=$(grep "char device redirected to /dev/pts/" /tmp/qemu-output.log | \
               grep "serial3" | \
               grep -o '/dev/pts/[0-9]*' | head -1)

    if [[ -z "$PTY_PATH" ]]; then
        # Fallback: get the last PTY from any serial line
        PTY_PATH=$(grep "char device redirected to /dev/pts/" /tmp/qemu-output.log | \
                   tail -1 | grep -o '/dev/pts/[0-9]*')
    fi

    if [[ -z "$PTY_PATH" ]]; then
        log_error "Could not find QEMU PTY path"
        exit 1
    fi

    log_ok "QEMU PTY: $PTY_PATH"

    # Wait for BMC to boot and SSH to become available
    log_info "Waiting for BMC to boot (this may take a few minutes)..."
    for i in $(seq 1 300); do
        if $SSH_CMD "echo ready" 2>/dev/null; then
            break
        fi
        if [[ $((i % 30)) -eq 0 ]]; then
            log_info "Still waiting for BMC... ($i seconds)"
        fi
        sleep 1
    done

    if ! $SSH_CMD "echo ready" 2>/dev/null; then
        log_error "BMC did not become reachable via SSH"
        exit 1
    fi
    log_ok "BMC is ready"
fi

# ===== Phase 2: Configure MCTP in Guest =====
log_step "Phase 2: Configure MCTP in Guest"

log_info "Setting up MCTP serial in guest on $GUEST_SERIAL"

# Set up MCTP serial link in the guest
$SSH_CMD "mctp link serial $GUEST_SERIAL &" 2>/dev/null || true
sleep 2

# Find the MCTP interface in the guest
GUEST_MCTP_IF=$($SSH_CMD "ls /sys/class/net/ | grep mctp | tail -1" 2>/dev/null)
if [[ -z "$GUEST_MCTP_IF" ]]; then
    log_error "No MCTP interface found in guest"
    log_info "Trying to load mctp-serial module in guest..."
    $SSH_CMD "modprobe mctp-serial 2>/dev/null; mctp link serial $GUEST_SERIAL &" 2>/dev/null || true
    sleep 2
    GUEST_MCTP_IF=$($SSH_CMD "ls /sys/class/net/ | grep mctp | tail -1" 2>/dev/null)
fi

if [[ -z "$GUEST_MCTP_IF" ]]; then
    log_error "Still no MCTP interface in guest. Check kernel config."
    exit 1
fi
log_ok "Guest MCTP interface: $GUEST_MCTP_IF"

# Configure MCTP addressing in the guest
log_info "Configuring MCTP addressing in guest"
$SSH_CMD "mctp link set $GUEST_MCTP_IF up" 2>/dev/null
$SSH_CMD "mctp addr add $BMC_EID dev $GUEST_MCTP_IF" 2>/dev/null
$SSH_CMD "mctp route add $FD_EID via $GUEST_MCTP_IF" 2>/dev/null
$SSH_CMD "mctp neigh add $FD_EID dev $GUEST_MCTP_IF lladdr 0x00" 2>/dev/null
log_ok "Guest MCTP addressing configured (BMC EID=$BMC_EID, route to FD EID=$FD_EID)"

# ===== Phase 3: Start pldm-sim on Host =====
log_step "Phase 3: Start PLDM Device Simulator"

log_info "Loading mctp-serial module on host"
modprobe mctp-serial 2>/dev/null || log_warn "mctp-serial module already loaded or not available"

log_info "Starting pldm-sim on $PTY_PATH"
export MCTP_TOOL="$MCTP_TOOL"
export LD_LIBRARY_PATH="${LIBPLDM_DIR}:${LD_LIBRARY_PATH:-}"

"$PLDM_SIM" "$PTY_PATH" > /tmp/pldm-sim.log 2>&1 &
PLDM_SIM_PID=$!
log_info "pldm-sim PID: $PLDM_SIM_PID"

# Wait for pldm-sim to initialize
sleep 3

if ! kill -0 "$PLDM_SIM_PID" 2>/dev/null; then
    log_error "pldm-sim exited prematurely"
    cat /tmp/pldm-sim.log
    exit 1
fi
log_ok "pldm-sim running"

# Show initial setup logs
log_info "pldm-sim setup output:"
cat /tmp/pldm-sim.log

# ===== Phase 4: Trigger Firmware Update =====
log_step "Phase 4: Trigger Firmware Update"

# Copy firmware package to guest
log_info "Copying firmware package to guest"
$SCP_CMD "$FW_PKG" root@localhost:/tmp/images/ 2>/dev/null

# Check if pldmd is running
if $SSH_CMD "systemctl is-active pldmd" 2>/dev/null | grep -q active; then
    log_ok "pldmd is active in guest"
else
    log_warn "pldmd may not be running, attempting to start..."
    $SSH_CMD "systemctl start pldmd" 2>/dev/null || true
    sleep 3
fi

# Start monitoring pldmd journal (background)
log_info "Starting pldmd journal monitor"
$SSH_CMD "journalctl -u pldmd -f --no-pager" > /tmp/pldmd-journal.log 2>&1 &
JOURNAL_PID=$!

# The firmware package in /tmp/images/ should trigger pldmd via inotify
# If not, we may need to use D-Bus activation
log_info "Firmware package deployed. Waiting for pldmd to process..."
log_info "(If pldmd doesn't auto-detect, D-Bus activation may be needed)"

# ===== Phase 5: Monitor and Verify =====
log_step "Phase 5: Monitor and Verify"

TIMEOUT=180
FOUND_CANNOT_UPDATE=false
FOUND_SKIPPING=false
FOUND_COMP1_UPDATE=false
FOUND_COMPLETION=false

log_info "Monitoring for expected firmware update events (timeout: ${TIMEOUT}s)..."

for i in $(seq 1 $TIMEOUT); do
    # Check pldm-sim log for events
    if grep -q "CANNOT BE UPDATED" /tmp/pldm-sim.log 2>/dev/null; then
        if [[ "$FOUND_CANNOT_UPDATE" == "false" ]]; then
            log_ok "CHECKPOINT 1: pldm-sim returned CANNOT_BE_UPDATED for component 100"
            FOUND_CANNOT_UPDATE=true
        fi
    fi

    # Check pldmd journal for skipping message
    if grep -qi "cannot be updated.*skipping\|skipping.*cannot" /tmp/pldmd-journal.log 2>/dev/null; then
        if [[ "$FOUND_SKIPPING" == "false" ]]; then
            log_ok "CHECKPOINT 2: pldmd logged 'cannot be updated...skipping' (FIX #2 VERIFIED)"
            FOUND_SKIPPING=true
        fi
    fi

    # Check if pldmd sent UpdateComponent for component 200
    if grep -q "UpdateComponent for comp 200: CAN BE UPDATED" /tmp/pldm-sim.log 2>/dev/null; then
        if [[ "$FOUND_COMP1_UPDATE" == "false" ]]; then
            log_ok "CHECKPOINT 3: pldmd advanced to component 200 (FIX #2 VERIFIED - state machine continued)"
            FOUND_COMP1_UPDATE=true
        fi
    fi

    # Check for firmware data transfer
    if grep -q "FirmwareData: comp 200" /tmp/pldm-sim.log 2>/dev/null; then
        if [[ "$FOUND_COMPLETION" == "false" ]]; then
            log_ok "CHECKPOINT 4: Firmware data transfer started for component 200 (FIX #3 VERIFIED - reqFwDataTimer created)"
            FOUND_COMPLETION=true
        fi
    fi

    # Check for verify/apply
    if grep -q "Verify: comp 200" /tmp/pldm-sim.log 2>/dev/null; then
        log_ok "CHECKPOINT 5: Verify phase reached for component 200"
    fi

    if grep -q "Apply: comp 200" /tmp/pldm-sim.log 2>/dev/null; then
        log_ok "CHECKPOINT 6: Apply phase reached for component 200"
    fi

    if grep -q "Activate:" /tmp/pldm-sim.log 2>/dev/null; then
        log_ok "CHECKPOINT 7: Activate phase reached - FULL UPDATE CYCLE COMPLETE"
        break
    fi

    # All key checkpoints reached?
    if [[ "$FOUND_CANNOT_UPDATE" == "true" ]] && \
       [[ "$FOUND_COMP1_UPDATE" == "true" ]] && \
       [[ "$FOUND_COMPLETION" == "true" ]]; then
        log_info "Key checkpoints reached, waiting for completion..."
    fi

    # Periodic status
    if [[ $((i % 15)) -eq 0 ]]; then
        log_info "Waiting... ($i/${TIMEOUT}s)"
        log_info "  pldm-sim events so far:"
        grep -c "PLDM msg from" /tmp/pldm-sim.log 2>/dev/null || echo "  (no PLDM messages)"
    fi

    sleep 1
done

# Kill journal monitor
kill "$JOURNAL_PID" 2>/dev/null || true

# ===== Results =====
log_step "Test Results"

echo ""
echo "===== pldm-sim Log ====="
cat /tmp/pldm-sim.log
echo ""
echo "===== pldmd Journal (relevant lines) ====="
grep -i "firmware\|component\|update\|cannot\|skip\|error\|fail" /tmp/pldmd-journal.log 2>/dev/null || echo "(no relevant journal entries)"
echo ""

# Summary
PASS=0
FAIL=0

check_result() {
    local desc="$1"
    local result="$2"
    if [[ "$result" == "true" ]]; then
        log_ok "PASS: $desc"
        ((PASS++))
    else
        log_error "FAIL: $desc"
        ((FAIL++))
    fi
}

echo ""
log_step "Verification Checklist"
check_result "Component 0 (ID 100) returned CANNOT_BE_UPDATED" "$FOUND_CANNOT_UPDATE"
check_result "pldmd logged skipping message (Fix #2)" "$FOUND_SKIPPING"
check_result "pldmd advanced to component 1 (ID 200)" "$FOUND_COMP1_UPDATE"
check_result "Firmware data transfer started for component 1 (Fix #3)" "$FOUND_COMPLETION"

echo ""
if [[ $FAIL -eq 0 ]] && [[ $PASS -gt 0 ]]; then
    log_ok "ALL CHECKS PASSED ($PASS/$((PASS+FAIL)))"
    exit 0
else
    log_error "SOME CHECKS FAILED ($PASS passed, $FAIL failed)"
    exit 1
fi
