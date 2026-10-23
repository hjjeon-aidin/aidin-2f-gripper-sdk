// Aidin Gripper SDK - implementation over the self-contained ModbusRtu
// transport (no external Modbus library).
#include "aidin/gripper.hpp"
#include "aidin/modbus_rtu.hpp"
#include "aidin/registers.hpp"

#include <array>
#include <map>
#include <mutex>
#include <optional>
#include <thread>

namespace aidin {

namespace {

constexpr auto kPollInterval = std::chrono::milliseconds(20);
// rHOM/rATR are rising-edge triggered and rGTO re-execution needs a 0->1
// pulse (protocol.md §1, v1.9.1) — hold each edge for this long.
constexpr auto kEdgePulse = std::chrono::milliseconds(20);

GripperStage stageOf(uint16_t st) {
    switch (st & reg::GSTA_MASK) {
        case reg::GSTA_INIT:   return GripperStage::Activating;
        case reg::GSTA_ACTIVE: return GripperStage::Active;
        default:               return GripperStage::Reset;
    }
}

ObjectState objectOf(uint16_t st) {
    switch (st & reg::GOBJ_MASK) {
        case reg::GOBJ_OPEN:   return ObjectState::OpenContact;
        case reg::GOBJ_CLOSE:  return ObjectState::ClosedContact;
        case reg::GOBJ_TARGET: return ObjectState::AtTarget;
        default:               return ObjectState::Moving;
    }
}

TactileTip tactileFlags(uint16_t st) {
    TactileTip t;
    t.status    = st;
    t.valid     = (st & reg::TACT_ST_VALID)      != 0;
    t.saturated = (st & reg::TACT_ST_SATURATED)  != 0;
    t.timeout   = (st & reg::TACT_ST_TIMEOUT)    != 0;
    t.connected = (st & reg::TACT_ST_CONNECTED)  != 0;
    t.rawStream = (st & reg::TACT_ST_RAW_STREAM) != 0;
    t.contact   = (st & reg::TACT_ST_CONTACT)    != 0;
    t.seen      = (st & reg::TACT_ST_SEEN)       != 0;
    t.rawFresh  = (st & reg::TACT_ST_RAW_FRESH)  != 0;
    return t;
}

// One tip of the fast block: (Px8,Py8) (Fx8,Fy8) (Fz8,status8), high byte first.
TactileTip fastTip(const uint16_t* w) {
    TactileTip t = tactileFlags(static_cast<uint16_t>((w[2] & 0xFF) << 8));  // status8 = TACT_STATUS[15:8]
    const auto s8 = [](uint16_t b) { return static_cast<float>(static_cast<int8_t>(b & 0xFF)); };
    const auto u8 = [](uint16_t b) { return static_cast<float>(b & 0xFF); };
    t.posX = u8(w[0] >> 8) * reg::FAST_PX_MAX  / 255.0f;
    t.posY = s8(w[0])      * reg::FAST_PY_MAX  / 127.0f;
    t.fx   = s8(w[1] >> 8) * reg::FAST_FXY_MAX / 127.0f;
    t.fy   = s8(w[1])      * reg::FAST_FXY_MAX / 127.0f;
    t.fz   = u8(w[2] >> 8) * reg::FAST_FZ_MAX  / 255.0f;
    return t;
}

std::string hexAddr(uint16_t addr) {
    static const char* digits = "0123456789ABCDEF";
    std::string s = "0x";
    for (int shift = 12; shift >= 0; shift -= 4) {
        s += digits[(addr >> shift) & 0xF];
    }
    return s;
}

// One physical serial connection, shared by every Gripper on it.
//
// ModbusRtu is not thread-safe and RS485 is a half-duplex multi-drop bus
// (only one transaction may be in flight at a time), so several Gripper
// instances - each with its own slave ID - share one SharedBus instead of
// each trying to open the OS serial handle for the same port (which fails
// outright on Windows and can silently corrupt the bus on Linux).
struct SharedBus {
    ModbusRtu  bus;
    std::mutex mutex;
    int        baudrate = 0;
    char       parity   = 'N';
};

std::mutex& registryMutex() {
    static std::mutex m;
    return m;
}

// weak_ptr: once every Gripper sharing a port disconnects, the SharedBus is
// destroyed (closing the port via ~ModbusRtu) without any manual refcount
// bookkeeping; a stale/expired entry is simply replaced on the next connect().
std::map<std::string, std::weak_ptr<SharedBus>>& portRegistry() {
    static std::map<std::string, std::weak_ptr<SharedBus>> registry;
    return registry;
}

} // namespace

struct Gripper::Impl {
    std::shared_ptr<SharedBus> shared;
    uint8_t   slaveId      = 1;
    uint16_t  lastAction   = 0;   // last ACTION written (rGTO pulse decision)
    int       lastPosition = -1;  // last rPR written, -1 = none yet
    // Set by setResponseTimeout() before the port that creates it is open;
    // applied once connect() actually opens a fresh SharedBus.
    std::optional<std::chrono::milliseconds> pendingTimeout;

    void write1(uint16_t addr, uint16_t value) {
        if (!shared) throw GripperError("not connected");
        {
            std::lock_guard<std::mutex> lock(shared->mutex);
            if (!shared->bus.writeSingle(slaveId, addr, value)) {
                throw GripperError("write " + hexAddr(addr) + " failed: " + shared->bus.lastError());
            }
        }
        if (addr == reg::ACTION)   lastAction   = value;
        if (addr == reg::POSITION) lastPosition = static_cast<int>(value & 0xFF);
    }

    uint16_t read1(uint16_t addr) {
        if (!shared) throw GripperError("not connected");
        uint16_t v = 0;
        std::lock_guard<std::mutex> lock(shared->mutex);
        if (!shared->bus.readHolding(slaveId, addr, 1, &v)) {
            throw GripperError("read " + hexAddr(addr) + " failed: " + shared->bus.lastError());
        }
        return v;
    }

    void readBlock(uint16_t addr, uint16_t count, uint16_t* out) {
        if (!shared) throw GripperError("not connected");
        std::lock_guard<std::mutex> lock(shared->mutex);
        if (!shared->bus.readHolding(slaveId, addr, count, out)) {
            throw GripperError("read block " + hexAddr(addr) + " failed: " + shared->bus.lastError());
        }
    }

    void writeBlock(uint16_t addr, const uint16_t* values, uint16_t count) {
        if (!shared) throw GripperError("not connected");
        std::lock_guard<std::mutex> lock(shared->mutex);
        if (!shared->bus.writeMultiple(slaveId, addr, values, count)) {
            throw GripperError("write block " + hexAddr(addr) + " failed: " + shared->bus.lastError());
        }
    }

    // Issue an instant DEV_CMD and wait for OK; throw on ERROR or timeout.
    void runDevCmd(uint16_t code, const std::string& name,
                   const std::string& rejectHint = "DEV_STATUS=ERROR") {
        write1(reg::DEV_CMD, code);
        // The firmware runs DEV_CMD in its main loop and only overwrites
        // DEV_STATUS with the result (never resets it on write) — give it a
        // moment so we don't read back a previous command's status.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (std::chrono::steady_clock::now() < deadline) {
            const uint16_t dev = read1(reg::DEV_STATUS);
            if (dev == reg::DEV_STATUS_OK) return;
            if (dev == reg::DEV_STATUS_ERROR) {
                throw GripperError(name + " rejected (" + rejectHint + ")");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        throw GripperError(name + " timeout");
    }
};

Gripper::Gripper() : p_(std::make_unique<Impl>()) {}
Gripper::~Gripper() = default;
Gripper::Gripper(Gripper&&) noexcept = default;
Gripper& Gripper::operator=(Gripper&&) noexcept = default;

void Gripper::connect(const std::string& port,
                      int  baudrate,
                      char parity,
                      int  slaveId)
{
    if (slaveId < 1 || slaveId > 247) {
        throw GripperError("slave id out of range (1..247)");
    }

    std::shared_ptr<SharedBus> shared;
    {
        std::lock_guard<std::mutex> lock(registryMutex());
        auto& registry = portRegistry();
        auto  it = registry.find(port);
        if (it != registry.end()) shared = it->second.lock();

        if (shared) {
            // Another Gripper already has this port open (e.g. a second
            // device at a different slaveId on the same RS485 bus) -
            // baudrate/parity are properties of the shared physical link,
            // not of one slave, and must match.
            if (shared->baudrate != baudrate || shared->parity != parity) {
                throw GripperError(
                    "'" + port + "' is already open at " + std::to_string(shared->baudrate) +
                    " baud parity='" + std::string(1, shared->parity) +
                    "'; every device on one RS485 bus must share the same baudrate/parity");
            }
        } else {
            shared = std::make_shared<SharedBus>();
            if (!shared->bus.open(port, baudrate, parity)) {
                throw GripperError("connect failed: " + shared->bus.lastError());
            }
            if (p_->pendingTimeout) {
                shared->bus.setResponseTimeout(*p_->pendingTimeout);
            }
            shared->baudrate = baudrate;
            shared->parity   = parity;
            registry[port]   = shared;
        }
    }

    p_->shared       = shared;
    p_->slaveId      = static_cast<uint8_t>(slaveId);
    p_->lastAction   = 0;
    p_->lastPosition = -1;
}

void Gripper::disconnect() noexcept { p_->shared.reset(); }
bool Gripper::isConnected() const noexcept { return p_->shared && p_->shared->bus.isOpen(); }

void Gripper::setResponseTimeout(std::chrono::milliseconds t) {
    if (p_->shared) {
        p_->shared->bus.setResponseTimeout(t);
    } else {
        // Not connected yet - remember it and apply once connect() opens
        // (or joins) a port.
        p_->pendingTimeout = t;
    }
}

void Gripper::activate(std::chrono::milliseconds timeout) {
    p_->write1(reg::ACTION, reg::RACT);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (readState().stage == GripperStage::Active) return;
        std::this_thread::sleep_for(kPollInterval);
    }
    throw GripperError("activate timeout (gSTA never reached Active)");
}

void Gripper::deactivate() {
    p_->write1(reg::ACTION, 0);
}

void Gripper::stop() {
    // rGTO=0 stops an in-flight motion; the position regulator keeps holding.
    p_->write1(reg::ACTION, reg::RACT);
}

void Gripper::home(std::chrono::milliseconds timeout) {
    // Activation must be set; firmware homes on rising edge of rHOM.
    p_->write1(reg::ACTION, reg::RACT);            // ensure rHOM is low
    std::this_thread::sleep_for(kEdgePulse);
    p_->write1(reg::ACTION, reg::RACT | reg::RHOM);

    // Let firmware observe the rising edge and clear gHOM before we poll —
    // otherwise we might briefly see the previous-cycle gHOM=1 and return early.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto s = readState();
        if (s.fault) {
            throw GripperError("homing fault code=" + std::to_string(s.fault));
        }
        if (s.homed) {
            // clear rHOM so next call sees a fresh rising edge
            p_->write1(reg::ACTION, reg::RACT);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    p_->write1(reg::ACTION, reg::RACT);            // don't leave rHOM latched
    throw GripperError("home timeout");
}

void Gripper::fullInit(std::chrono::milliseconds timeout) {
    // DH-compatible full init: writing exactly 0xA5 to ACTION triggers stroke
    // calibration. The trigger is edge-detected on the written VALUE, so write
    // plain rACT first to guarantee a fresh edge regardless of prior state.
    p_->write1(reg::ACTION, reg::RACT);
    std::this_thread::sleep_for(kEdgePulse);
    p_->write1(reg::ACTION, reg::ACTION_FULL_INIT);

    // DEV_STATUS may still hold a previous command's OK — only trust a
    // terminal value after RUNNING has been observed. A rejection (already
    // homing/calibrating) reports ERROR without ever showing RUNNING.
    bool sawRunning = false;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto s = readState();
        if (s.fault) {
            throw GripperError("full-init fault code=" + std::to_string(s.fault));
        }
        const uint16_t dev = p_->read1(reg::DEV_STATUS);
        if (dev == reg::DEV_STATUS_RUNNING) sawRunning = true;
        if (dev == reg::DEV_STATUS_ERROR) {
            p_->write1(reg::ACTION, reg::RACT);
            throw GripperError("full-init rejected or failed (DEV_STATUS=ERROR)");
        }
        if (sawRunning && dev == reg::DEV_STATUS_OK && s.homed) {
            p_->write1(reg::ACTION, reg::RACT);    // fresh value edge for next call
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    p_->write1(reg::ACTION, reg::RACT);
    throw GripperError("full-init timeout");
}

void Gripper::setTrajLimits(uint16_t maxVel, uint16_t maxAcc) {
    p_->write1(reg::TRAJ_MAX_VEL, maxVel);
    p_->write1(reg::TRAJ_MAX_ACC, maxAcc);
    p_->runDevCmd(reg::DEV_CMD_TRAJ_APPLY, "traj-apply",
                  "vel 1..150, acc 1..800, not during homing/cal");
}

void Gripper::moveTo(uint8_t position,
                     uint8_t speed,
                     uint8_t force,
                     bool    blocking,
                     std::chrono::milliseconds timeout)
{
    p_->write1(reg::SPEED, speed);
    p_->write1(reg::FORCE, force);

    // Firmware executes goto only on an rGTO rising edge OR an rPR change
    // (protocol.md §1, v1.9.1). Re-issuing the same position while rGTO is
    // already high would be silently ignored — pulse rGTO 0->1 instead.
    const bool samePos = (p_->lastPosition == static_cast<int>(position));
    if (samePos && (p_->lastAction & reg::RGTO)) {
        p_->write1(reg::ACTION, reg::RACT);
        std::this_thread::sleep_for(kEdgePulse);
    } else if (!samePos) {
        p_->write1(reg::POSITION, position);       // rPR first, ACTION last
    }
    p_->write1(reg::ACTION, reg::RACT | reg::RGTO);

    if (!blocking) return;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    // Give the gripper a moment to leave the previous AtTarget state.
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    while (std::chrono::steady_clock::now() < deadline) {
        const auto s = readState();
        if (s.fault) {
            throw GripperError("move fault code=" + std::to_string(s.fault));
        }
        if (s.object != ObjectState::Moving) return;
        std::this_thread::sleep_for(kPollInterval);
    }
    throw GripperError("moveTo timeout");
}

void Gripper::emergencyRelease(bool closeDirection) {
    // rATR is a rising edge processed only while rACT=1 (firmware ignores all
    // commands with rACT low) — keep rACT set and pulse rATR. Writing rATR
    // alone would just deactivate the motor without releasing, and would
    // poison the firmware's edge tracking for the next attempt.
    uint16_t bits = reg::RACT | reg::RATR;
    if (closeDirection) bits |= reg::RADR;
    p_->write1(reg::ACTION, bits);
    std::this_thread::sleep_for(kEdgePulse);
    p_->write1(reg::ACTION, reg::RACT);
}

GripperState Gripper::readState() {
    // Read the full status block (0x0200 .. 0x0207) in one transaction.
    std::array<uint16_t, 8> buf{};
    p_->readBlock(reg::STATUS, static_cast<uint16_t>(buf.size()), buf.data());

    GripperState s;
    const uint16_t st = buf[0];
    s.activated  = (st & reg::GACT) != 0;
    s.homed      = (st & reg::GHOM) != 0;
    s.goToActive = (st & reg::GGTO) != 0;
    s.stage      = stageOf(st);
    s.object     = objectOf(st);

    s.fault        = static_cast<uint8_t>(buf[1] & 0xFF);
    s.positionEcho = static_cast<uint8_t>(buf[2] & 0xFF);
    s.position     = static_cast<uint8_t>(buf[3] & 0xFF);
    s.current      = static_cast<uint8_t>(buf[4] & 0xFF);
    s.speed        = static_cast<uint8_t>(buf[5] & 0xFF);
    s.voltage      = static_cast<float>(buf[6]) * 0.1f;
    s.latchedFault = static_cast<uint8_t>(buf[7] & 0xFF);
    return s;
}

bool Gripper::isAtTarget() { return readState().object == ObjectState::AtTarget; }
bool Gripper::hasFault()   { return readState().fault != 0; }

FastState Gripper::readFastState() {
    // 0x02C0..0x02C9: [STATUS][gPO][gCU][right x3][left x3][gFLT], one transaction.
    std::array<uint16_t, reg::FAST_WORDS> buf{};
    p_->readBlock(reg::FAST_BLOCK, static_cast<uint16_t>(buf.size()), buf.data());

    FastState s;
    const uint16_t st = buf[0];
    s.activated  = (st & reg::GACT) != 0;
    s.homed      = (st & reg::GHOM) != 0;
    s.goToActive = (st & reg::GGTO) != 0;
    s.stage      = stageOf(st);
    s.object     = objectOf(st);
    s.position   = static_cast<uint8_t>(buf[1] & 0xFF);
    s.current    = static_cast<uint8_t>(buf[2] & 0xFF);
    s.right      = fastTip(&buf[3]);
    s.left       = fastTip(&buf[6]);
    s.fault      = static_cast<uint8_t>(buf[9] & 0xFF);
    return s;
}

TactilePair Gripper::readTactile() {
    // 0x0220..0x022F: per tip [status][posX][posY][fx][fy][fz][rate][rxcnt].
    std::array<uint16_t, 2 * reg::TACT_TIP_WORDS> buf{};
    p_->readBlock(reg::TACT1_STATUS, static_cast<uint16_t>(buf.size()), buf.data());

    const auto s16 = [](uint16_t v) { return static_cast<float>(static_cast<int16_t>(v)); };
    TactilePair tips;
    for (size_t i = 0; i < tips.size(); ++i) {
        const uint16_t* w = &buf[i * reg::TACT_TIP_WORDS];
        TactileTip t = tactileFlags(w[0]);
        t.posX    = s16(w[1]) / reg::TACT_POS_SCALE;
        t.posY    = s16(w[2]) / reg::TACT_POS_SCALE;
        t.fx      = s16(w[3]) / reg::TACT_FORCE_SCALE;
        t.fy      = s16(w[4]) / reg::TACT_FORCE_SCALE;
        t.fz      = s16(w[5]) / reg::TACT_FORCE_SCALE;
        t.rateHz  = w[6];
        t.rxCount = w[7];
        tips[i] = t;
    }
    return tips;
}

TactileRaw Gripper::readTactileRaw(uint16_t tip) {
    if (tip != reg::TACT_TIP_RIGHT && tip != reg::TACT_TIP_LEFT) {
        throw GripperError("readTactileRaw: tip must be TACT_TIP_RIGHT or TACT_TIP_LEFT");
    }
    std::array<uint16_t, reg::TACT_RAW_WORDS> buf{};
    p_->readBlock(tip == reg::TACT_TIP_RIGHT ? reg::TACT1_RAW : reg::TACT2_RAW,
                  static_cast<uint16_t>(buf.size()), buf.data());

    TactileRaw r;
    r.sets = buf[0];
    r.diag = buf[1];
    for (size_t i = 0; i < r.channels.size(); ++i) {
        r.channels[i] = static_cast<int16_t>(buf[2 + i]);
    }
    for (size_t g = 0; g < r.groups.size(); ++g) {
        r.groups[g] = r.channels[3 * g] + r.channels[3 * g + 1] + r.channels[3 * g + 2];
    }
    return r;
}

void Gripper::tactileZero(uint16_t tips) {
    tactileCommand(reg::TACT_SCMD_ZERO, tips);
}

void Gripper::tactileCommand(uint16_t code, uint16_t tips, uint16_t arg) {
    if (tips < 1 || tips > reg::TACT_TIP_BOTH) {
        throw GripperError("tactileCommand: tips must be a TACT_TIP_* bitmask (1..3)");
    }
    const uint16_t block[3] = {code, tips, arg};
    p_->writeBlock(reg::TACT_SCMD_CODE, block, 3);
    p_->runDevCmd(reg::DEV_CMD_TACTILE_CMD, "tactile command");
}

void Gripper::sensorZero() {
    p_->runDevCmd(reg::DEV_CMD_SENSOR_ZERO, "sensor-zero");
}

uint16_t Gripper::readRegister(uint16_t pduAddr) {
    return p_->read1(pduAddr);
}
void Gripper::writeRegister(uint16_t pduAddr, uint16_t value) {
    p_->write1(pduAddr, value);
}

} // namespace aidin
