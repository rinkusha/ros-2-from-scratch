// Wire protocol shared by the ROS 2 joint bridge (host) and the ESP32 joint
// controller (firmware). Both sides include this exact header, so the structs
// below ARE the protocol: change a field here and rebuild both targets.
//
// Transport: UART, 8N1. Every message is a packed struct below, followed by a
// CRC-16/CCITT-FALSE over the struct bytes (little-endian), COBS-encoded and
// terminated with a single 0x00 byte. See framing.hpp.
//
// Units: position = rad, velocity = rad/s, effort = normalised motor command
// in [-1, 1] (fraction of full PWM duty, sign = direction).
#pragma once

#include <cstddef>
#include <cstdint>

namespace arm {

constexpr uint8_t kProtocolVersion = 1;

// Upper bound on joints carried in one message. The firmware reports how many
// it actually drives in StateMsg::joint_count; the host must not send more.
constexpr std::size_t kMaxJoints = 6;

// Joint index meaning "all joints" in CommandMsg.
constexpr uint8_t kAllJoints = 0xFF;

enum class MsgType : uint8_t {
  // host -> mcu
  kSetpoint = 0x01,  // SetpointMsg, sent at the host control rate (100 Hz)
  kGains = 0x02,     // GainsMsg, one joint per message
  kCommand = 0x03,   // CommandMsg
  // mcu -> host
  kState = 0x81,  // StateMsg, sent at the firmware state rate (100 Hz)
  kAck = 0x82,    // AckMsg, reply to kGains / kCommand
  kLog = 0x83,    // LogMsg, human-readable text from firmware
};

enum class Command : uint8_t {
  kPing = 0,        // no-op, just elicits an Ack
  kEnable = 1,      // close the loop and drive the motor
  kDisable = 2,     // open the loop, motor coasts
  kZero = 3,        // set encoder count to 0 at the current pose (homing)
  kClearFault = 4,  // leave kFault -> kDisabled
};

// JointStatus is the controller mode of one joint.
enum class JointStatus : uint8_t {
  kDisabled = 0,  // motor off, PID not running
  kEnabled = 1,   // tracking host setpoints
  kHold = 2,      // setpoint watchdog tripped: holding last measured position
  kFault = 3,     // motor off until kClearFault
};

// Bit flags in StateMsg::fault[i].
enum FaultBits : uint8_t {
  kFaultNone = 0,
  kFaultSoftLimit = 1u << 0,   // measured position outside [pos_min, pos_max] + margin
  kFaultEncoder = 1u << 1,     // encoder unit error
  kFaultOverCurrent = 1u << 2, // reserved for a current sense hook
  kFaultSetpointNaN = 1u << 3, // host sent a non-finite setpoint
};

enum class AckResult : uint8_t {
  kOk = 0,
  kBadJoint = 1,
  kBadValue = 2,
  kRefused = 3,  // e.g. enable while in fault
  kUnknown = 4,
};

#pragma pack(push, 1)

struct Header {
  uint8_t version;  // kProtocolVersion
  uint8_t type;     // MsgType
  uint16_t seq;     // sender's monotonically increasing sequence number
};

// host -> mcu. One sample of the trajectory for every joint.
struct SetpointMsg {
  Header hdr;
  uint8_t joint_count;  // number of valid entries in the arrays below
  float position[kMaxJoints];   // rad
  float velocity[kMaxJoints];   // rad/s, used as feed-forward
  float effort_ff[kMaxJoints];  // [-1,1], e.g. gravity compensation; 0 if unused
};

// host -> mcu. Full controller configuration for one joint.
struct GainsMsg {
  Header hdr;
  uint8_t joint;
  float kp;             // effort per rad
  float ki;             // effort per rad*s
  float kd;             // effort per rad/s (applied to measured velocity)
  float kff_v;          // effort per rad/s of commanded velocity
  float i_limit;        // |integral term| clamp, in effort units
  float out_limit;      // |output| clamp, in effort units (<= 1)
  float vel_filter_hz;  // low-pass cutoff for measured velocity
  float pos_min;        // soft limit, rad
  float pos_max;        // soft limit, rad
};

// host -> mcu.
struct CommandMsg {
  Header hdr;
  uint8_t command;  // Command
  uint8_t joint;    // joint index or kAllJoints
};

// mcu -> host. Snapshot of every joint, produced by the 1 kHz loop.
struct StateMsg {
  Header hdr;
  uint32_t timestamp_us;       // firmware monotonic clock
  uint16_t last_setpoint_seq;  // seq of the most recent SetpointMsg applied
  uint16_t setpoint_age_ms;    // time since that setpoint arrived
  uint16_t loop_period_max_us; // worst-case control period since last StateMsg
  uint8_t joint_count;
  uint8_t status[kMaxJoints];  // JointStatus
  uint8_t fault[kMaxJoints];   // FaultBits
  float position[kMaxJoints];  // rad
  float velocity[kMaxJoints];  // rad/s (filtered)
  float effort[kMaxJoints];    // [-1,1] command actually applied
};

// mcu -> host.
struct AckMsg {
  Header hdr;
  uint8_t acked_type;  // MsgType that was acknowledged
  uint16_t acked_seq;  // its seq
  uint8_t result;      // AckResult
};

// mcu -> host. Not NUL-terminated; length comes from the frame.
struct LogMsg {
  Header hdr;
  uint8_t level;  // 0 error, 1 warn, 2 info
  char text[64];
};

#pragma pack(pop)

static_assert(sizeof(Header) == 4, "Header must be 4 bytes");
static_assert(sizeof(SetpointMsg) == 4 + 1 + 3 * 4 * kMaxJoints, "SetpointMsg layout");
static_assert(sizeof(GainsMsg) == 4 + 1 + 9 * 4, "GainsMsg layout");
static_assert(sizeof(CommandMsg) == 6, "CommandMsg layout");
static_assert(sizeof(StateMsg) == 4 + 4 + 2 + 2 + 2 + 1 + 2 * kMaxJoints + 3 * 4 * kMaxJoints, "StateMsg layout");
static_assert(sizeof(AckMsg) == 8, "AckMsg layout");

// Largest payload any side has to buffer, before CRC and COBS.
constexpr std::size_t kMaxPayloadSize = sizeof(StateMsg) > sizeof(SetpointMsg) ? sizeof(StateMsg) : sizeof(SetpointMsg);

template <typename T>
inline void initHeader(T& msg, MsgType type, uint16_t seq) {
  msg.hdr.version = kProtocolVersion;
  msg.hdr.type = static_cast<uint8_t>(type);
  msg.hdr.seq = seq;
}

}  // namespace arm
