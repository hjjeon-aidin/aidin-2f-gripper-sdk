// Aidin Gripper SDK - C++ class wrapper
//
// Talks to AIDIN BLDC firmware (DH-Robotics compatible Modbus RTU) over RS485.
// Default link: 115200 8N1, slave ID 1.
//
// Quick start:
//   aidin::Gripper g;
//   g.connect("/dev/ttyUSB0");          // Windows: "COM3"
//   g.activate();
//   g.home();
//   g.moveTo(255, 200, 128, /*blocking=*/true);
#pragma once

#include "aidin/types.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

namespace aidin {

class GripperError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Gripper {
public:
    Gripper();
    ~Gripper();
    Gripper(const Gripper&)            = delete;
    Gripper& operator=(const Gripper&) = delete;
    Gripper(Gripper&&) noexcept;
    Gripper& operator=(Gripper&&) noexcept;

    // ---------- Connection ----------
    // port: "/dev/ttyUSB0" on Linux, "COM3" / "COM10" on Windows (auto-prefixed)
    // parity: 'N', 'E', or 'O'.  Default 'N' matches firmware default.
    void connect(const std::string& port,
                 int  baudrate = 115200,
                 char parity   = 'N',
                 int  slaveId  = 1);
    void disconnect() noexcept;
    bool isConnected() const noexcept;
    void setResponseTimeout(std::chrono::milliseconds t);

    // ---------- High-level commands ----------
    // Set rACT=1 and wait until gSTA reports Active (or throw on timeout).
    void activate(std::chrono::milliseconds timeout = std::chrono::seconds(3));
    // Set rACT=0. Does not wait.
    void deactivate();
    // Stop an in-flight motion (rGTO=0). Motor stays activated and holding.
    void stop();
    // Trigger homing (rising edge of rHOM) and block until gHOM=1.
    void home(std::chrono::milliseconds timeout = std::chrono::seconds(15));
    // Full initialization (protocol v1.12.0, DH-compatible ACTION=0xA5):
    // activate + stroke calibration — open homing, close-contact stroke
    // measurement, flash-persisted (readable at reg::CFG_STROKE). Works from
    // the inactive state. Blocks until DEV_STATUS=OK and gHOM=1; throws on
    // rejection (already homing/calibrating), fault, or timeout.
    // Fingers sweep the FULL stroke twice — keep the workspace clear.
    void fullInit(std::chrono::milliseconds timeout = std::chrono::seconds(30));
    // Set trajectory generator limits (protocol v1.11.0, RAM only — reboot
    // restores firmware defaults 100/400). rSP scales against these.
    //   maxVel [rad/s] 1..150, maxAcc [rad/s^2] 1..800
    // Throws if firmware rejects (out of range, or homing/calibrating).
    void setTrajLimits(uint16_t maxVel, uint16_t maxAcc);
    // Move to absolute position. 0=fully open, 255=fully closed.
    //   speed:  0..255   force/current limit: 0..255
    //   blocking=true waits until gOBJ != Moving (target reached or object hit).
    void moveTo(uint8_t position,
                uint8_t speed   = 128,
                uint8_t force   = 128,
                bool    blocking = false,
                std::chrono::milliseconds timeout = std::chrono::seconds(5));
    // Emergency release. dir=false opens, true closes. Sets rATR; user must
    // clear by calling activate() again.
    void emergencyRelease(bool closeDirection = false);

    // ---------- Status ----------
    GripperState readState();
    bool         isAtTarget();
    bool         hasFault();
    // Fast-poll block (0x02C0..0x02C9) in ONE transaction: status, position,
    // current, fault + both fingertips' tactile resultant, refreshed atomically
    // by the firmware (~4 ms at 115200 bps). Tactile values are 8-bit compressed
    // over each field's full scale (Px 32 mm, Py +-12.5 mm, Fx/Fy +-35 N, Fz 70 N).
    // Firmware protocol >= v1.15.0; the scales decode correctly only with >= v1.19.0.
    FastState    readFastState();

    // ---------- Tactile sensor ----------
    // Needs CFG_SENSOR_TYPE = reg::SENSOR_TACTILE (write, SAVE_CONFIG, power cycle).
    // Both fingertips' contact resultant (0x0220..0x022F, one transaction):
    // {right (tip 1), left (tip 2)} — contact point [mm], Fx/Fy/Fz [N], 0.01
    // resolution, link state. With another sensor type both read NotDetected.
    TactilePair  readTactile();
    // Raw 69-channel diagnostic stream of one tip (bring-up only).
    // tip = reg::TACT_TIP_RIGHT or reg::TACT_TIP_LEFT. The block keeps its last
    // values when the stream stops — trust it only while that tip's rawFresh.
    TactileRaw   readTactileRaw(uint16_t tip);
    // Sensor no-load zero (structured command ZERO, firmware >= v1.17.0).
    // Keep the pads untouched. After OK the tip reads Waiting ~1.25 s, then Live;
    // a tip whose zero CAN ID is unknown falls back to the firmware software tare.
    // Throws if rejected (not tactile, values invalid, sensor-feedback force control).
    void         tactileZero(uint16_t tips = 0x03 /* reg::TACT_TIP_BOTH */);
    // Structured tactile command (reg::TACT_SCMD_*); tips = reg::TACT_TIP_* bitmask.
    // Returns once the firmware reports OK (CAN frame sent; the sensor does not ack).
    void         tactileCommand(uint16_t code, uint16_t tips, uint16_t arg = 0);
    // Firmware software tare (DEV_CMD 0x08): current Fx/Fy/Fz of every valid tip
    // becomes zero (AFT F/T or tactile). RAM only — cleared on reboot.
    void         sensorZero();

    // ---------- Low-level escape hatch ----------
    // Direct Modbus register access. pduAddr is the protocol-level address
    // (e.g. 0x0100, 0x0200, 0x0182). Mostly useful for installer / factory
    // tooling that needs to read/write configuration registers.
    uint16_t readRegister(uint16_t pduAddr);
    void     writeRegister(uint16_t pduAddr, uint16_t value);

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace aidin
