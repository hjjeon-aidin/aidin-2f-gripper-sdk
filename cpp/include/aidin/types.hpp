// Aidin Gripper SDK - public types
#pragma once

#include <array>
#include <cstdint>
#include <ostream>

namespace aidin {

enum class GripperStage : uint8_t {
    Reset      = 0,
    Activating = 1,
    Active     = 3,
};

enum class ObjectState : uint8_t {
    Moving        = 0,
    OpenContact   = 1,  // object detected while opening
    ClosedContact = 2,  // object detected while closing
    AtTarget      = 3,
};

struct GripperState {
    bool         activated    = false;  // gACT
    bool         homed        = false;  // gHOM
    bool         goToActive   = false;  // gGTO
    GripperStage stage        = GripperStage::Reset;
    ObjectState  object       = ObjectState::Moving;
    uint8_t      fault        = 0;      // gFLT (0 = OK)
    uint8_t      positionEcho = 0;      // gPR
    uint8_t      position     = 0;      // gPO  0=open, 255=close
    uint8_t      current      = 0;      // gCU
    uint8_t      speed        = 0;      // gSP
    float        voltage      = 0.0f;   // V
    uint8_t      latchedFault = 0;      // gFLTO
};

// Per-tip tactile connection state, derived from the TACT_STATUS bits.
enum class TactileLink : uint8_t {
    NotDetected  = 0,  // never seen since boot: not fitted, wiring, CAN ID mismatch, or sensor type != tactile
    Disconnected = 1,  // was seen, frames stopped
    RawStream    = 2,  // connected, but the sensor firmware sends raw channels, not results
    Waiting      = 3,  // results arrive but are invalid: sensor boot zero (~4.5 s) or after a zero (~1.25 s)
    Live         = 4,  // valid values
};

// Contact resultant of one fingertip, in the gripper frame (firmware >= v1.18.0).
//   posX: 0..32 mm from the wrist-side edge of the cell area toward the
//         fingertip; 0 = no contact point (also when Fz is too small to locate one)
//   posY: +-12.5 mm across the pad width. Forces after tare, fz > 0 = pressing.
// Values are 0 while valid is false.
struct TactileTip {
    bool     valid     = false;
    bool     saturated = false;
    bool     timeout   = false;
    bool     connected = false;
    bool     rawStream = false;
    bool     contact   = false;  // valid and Fz >= 0.5 N
    bool     seen      = false;
    bool     rawFresh  = false;
    float    posX = 0.0f;        // mm
    float    posY = 0.0f;        // mm
    float    fx   = 0.0f;        // N
    float    fy   = 0.0f;        // N
    float    fz   = 0.0f;        // N
    uint16_t rateHz  = 0;        // result frames in the last second; readTactile() only
    uint16_t rxCount = 0;        // result frames since boot (wraps); readTactile() only
    uint16_t status  = 0;        // raw TACT_STATUS word (readTactile) or status8 << 8 (readFastState)

    TactileLink link() const {
        if (valid)     return TactileLink::Live;
        if (connected) return rawStream ? TactileLink::RawStream : TactileLink::Waiting;
        return seen ? TactileLink::Disconnected : TactileLink::NotDetected;
    }
    bool hasContactPoint() const { return valid && posX > 0.0f; }
};

// Both fingertips. Index 0 = right (tip 1), 1 = left (tip 2).
using TactilePair = std::array<TactileTip, 2>;

// Raw 69-channel diagnostic stream of one tip (bring-up only, unitless).
struct TactileRaw {
    uint16_t                 sets = 0;      // complete sets received (wraps)
    uint16_t                 diag = 0;      // sensor AFE diagnostic word
    std::array<int16_t, 69>  channels{};    // C0..C68, int16 deltas, ~0 unloaded
    std::array<int32_t, 23>  groups{};      // G1..G23 = sum of 3 channels each
};

// One snapshot of the fast-poll block 0x02C0..0x02C9 (single FC03 read).
// Gripper fields are copies of 0x0200/0x0201/0x0203/0x0204. Tactile values
// are 8-bit compressed (0.2 mm / 0.2 N per LSB) — use readTactile() when you
// need 0.01 resolution, rateHz or rxCount.
struct FastState {
    bool         activated  = false;
    bool         homed      = false;
    bool         goToActive = false;
    GripperStage stage      = GripperStage::Reset;
    ObjectState  object     = ObjectState::Moving;
    uint8_t      position   = 0;   // gPO
    uint8_t      current    = 0;   // gCU
    uint8_t      fault      = 0;   // gFLT
    TactileTip   right;            // tip 1
    TactileTip   left;             // tip 2
};

inline const char* toString(GripperStage s) {
    switch (s) {
        case GripperStage::Reset:      return "Reset";
        case GripperStage::Activating: return "Activating";
        case GripperStage::Active:     return "Active";
    }
    return "Unknown";
}

inline const char* toString(ObjectState o) {
    switch (o) {
        case ObjectState::Moving:        return "Moving";
        case ObjectState::OpenContact:   return "OpenContact";
        case ObjectState::ClosedContact: return "ClosedContact";
        case ObjectState::AtTarget:      return "AtTarget";
    }
    return "Unknown";
}

inline std::ostream& operator<<(std::ostream& os, const GripperState& s) {
    os << "{stage=" << toString(s.stage)
       << " obj="   << toString(s.object)
       << " pos="   << static_cast<int>(s.position)
       << " cur="   << static_cast<int>(s.current)
       << " spd="   << static_cast<int>(s.speed)
       << " V="     << s.voltage
       << " flt="   << static_cast<int>(s.fault)
       << "}";
    return os;
}

inline const char* toString(TactileLink l) {
    switch (l) {
        case TactileLink::NotDetected:  return "NotDetected";
        case TactileLink::Disconnected: return "Disconnected";
        case TactileLink::RawStream:    return "RawStream";
        case TactileLink::Waiting:      return "Waiting";
        case TactileLink::Live:         return "Live";
    }
    return "Unknown";
}

inline std::ostream& operator<<(std::ostream& os, const TactileTip& t) {
    os << "{" << toString(t.link())
       << " pos=(" << t.posX << "," << t.posY << ")mm"
       << " F=("   << t.fx << "," << t.fy << "," << t.fz << ")N";
    if (t.contact)   os << " contact";
    if (t.saturated) os << " SAT";
    os << "}";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const FastState& s) {
    os << "{stage=" << toString(s.stage)
       << " obj="   << toString(s.object)
       << " pos="   << static_cast<int>(s.position)
       << " cur="   << static_cast<int>(s.current)
       << " flt="   << static_cast<int>(s.fault)
       << " R="     << s.right
       << " L="     << s.left
       << "}";
    return os;
}

} // namespace aidin
