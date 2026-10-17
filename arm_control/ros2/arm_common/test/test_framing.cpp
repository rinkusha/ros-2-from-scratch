#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "arm_common/cobs.hpp"
#include "arm_common/crc16.hpp"
#include "arm_common/framing.hpp"
#include "arm_common/protocol.hpp"

using namespace arm;

TEST(Crc16, CheckValue) {
  const char* s = "123456789";
  EXPECT_EQ(crc16(reinterpret_cast<const uint8_t*>(s), 9), 0x29B1u);
}

TEST(Cobs, KnownVectors) {
  // From the COBS paper / Wikipedia.
  struct Case { std::vector<uint8_t> in, out; };
  const std::vector<Case> cases = {
      {{0x00}, {0x01, 0x01}},
      {{0x00, 0x00}, {0x01, 0x01, 0x01}},
      {{0x11, 0x22, 0x00, 0x33}, {0x03, 0x11, 0x22, 0x02, 0x33}},
      {{0x11, 0x22, 0x33, 0x44}, {0x05, 0x11, 0x22, 0x33, 0x44}},
      {{0x11, 0x00, 0x00, 0x00}, {0x02, 0x11, 0x01, 0x01, 0x01}},
  };
  for (const auto& c : cases) {
    uint8_t buf[64];
    const std::size_t n = cobsEncode(c.in.data(), c.in.size(), buf, sizeof(buf));
    ASSERT_EQ(n, c.out.size());
    EXPECT_EQ(std::memcmp(buf, c.out.data(), n), 0);
    for (std::size_t i = 0; i < n; ++i) EXPECT_NE(buf[i], 0);

    uint8_t dec[64];
    const std::size_t m = cobsDecode(buf, n, dec, sizeof(dec));
    ASSERT_EQ(m, c.in.size());
    EXPECT_EQ(std::memcmp(dec, c.in.data(), m), 0);
  }
}

TEST(Cobs, LongRunWithoutZeros) {
  std::vector<uint8_t> in(300);
  for (std::size_t i = 0; i < in.size(); ++i) in[i] = static_cast<uint8_t>(1 + (i % 254));
  uint8_t enc[cobsMaxEncodedSize(300)];
  const std::size_t n = cobsEncode(in.data(), in.size(), enc, sizeof(enc));
  ASSERT_GT(n, 0u);
  ASSERT_LE(n, cobsMaxEncodedSize(300));
  for (std::size_t i = 0; i < n; ++i) EXPECT_NE(enc[i], 0);
  uint8_t dec[300];
  ASSERT_EQ(cobsDecode(enc, n, dec, sizeof(dec)), 300u);
  EXPECT_EQ(std::memcmp(dec, in.data(), 300), 0);
}

TEST(Cobs, RejectsEmbeddedZero) {
  const uint8_t bad[] = {0x03, 0x11, 0x00};
  uint8_t dec[8];
  EXPECT_EQ(cobsDecode(bad, sizeof(bad), dec, sizeof(dec)), 0u);
}

TEST(Framing, SetpointRoundTrip) {
  SetpointMsg sp{};
  initHeader(sp, MsgType::kSetpoint, 1234);
  sp.joint_count = 6;
  for (std::size_t i = 0; i < kMaxJoints; ++i) {
    sp.position[i] = 0.1f * static_cast<float>(i);
    sp.velocity[i] = -0.5f + 0.25f * static_cast<float>(i);
    sp.effort_ff[i] = 0.0f;
  }

  uint8_t frame[kMaxFrameSize];
  const std::size_t n = encodeFrame(sp, frame, sizeof(frame));
  ASSERT_GT(n, sizeof(SetpointMsg) + kCrcSize);
  EXPECT_EQ(frame[n - 1], kFrameDelimiter);
  for (std::size_t i = 0; i + 1 < n; ++i) EXPECT_NE(frame[i], kFrameDelimiter);

  FrameParser parser;
  bool got = false;
  for (std::size_t i = 0; i < n; ++i) {
    const bool done = parser.feed(frame[i]);
    if (done) {
      EXPECT_EQ(i, n - 1);
      got = true;
    }
  }
  ASSERT_TRUE(got);
  ASSERT_TRUE(parser.is<SetpointMsg>(MsgType::kSetpoint));
  const SetpointMsg back = parser.as<SetpointMsg>();
  EXPECT_EQ(back.hdr.seq, 1234);
  EXPECT_EQ(back.joint_count, 6);
  for (std::size_t i = 0; i < kMaxJoints; ++i) {
    EXPECT_FLOAT_EQ(back.position[i], sp.position[i]);
    EXPECT_FLOAT_EQ(back.velocity[i], sp.velocity[i]);
  }
  EXPECT_EQ(parser.framesOk(), 1u);
  EXPECT_EQ(parser.framesBad(), 0u);
}

TEST(Framing, CorruptedByteIsRejected) {
  CommandMsg cmd{};
  initHeader(cmd, MsgType::kCommand, 7);
  cmd.command = static_cast<uint8_t>(Command::kEnable);
  cmd.joint = kAllJoints;

  uint8_t frame[kMaxFrameSize];
  const std::size_t n = encodeFrame(cmd, frame, sizeof(frame));
  ASSERT_GT(n, 0u);
  frame[2] ^= 0x40;  // flip a bit in the body

  FrameParser parser;
  bool got = false;
  for (std::size_t i = 0; i < n; ++i) got |= parser.feed(frame[i]);
  EXPECT_FALSE(got);
  EXPECT_EQ(parser.framesBad(), 1u);
}

TEST(Framing, ResyncsAfterGarbage) {
  // Garbage, then a delimiter, then a good frame: the parser must recover.
  AckMsg ack{};
  initHeader(ack, MsgType::kAck, 9);
  ack.acked_type = static_cast<uint8_t>(MsgType::kCommand);
  ack.acked_seq = 7;
  ack.result = static_cast<uint8_t>(AckResult::kOk);

  uint8_t frame[kMaxFrameSize];
  const std::size_t n = encodeFrame(ack, frame, sizeof(frame));
  ASSERT_GT(n, 0u);

  FrameParser parser;
  const uint8_t garbage[] = {0x55, 0xAA, 0x12, 0x00};
  for (uint8_t b : garbage) EXPECT_FALSE(parser.feed(b));
  bool got = false;
  for (std::size_t i = 0; i < n; ++i) got |= parser.feed(frame[i]);
  ASSERT_TRUE(got);
  ASSERT_TRUE(parser.is<AckMsg>(MsgType::kAck));
  EXPECT_EQ(parser.as<AckMsg>().acked_seq, 7);
}

TEST(Framing, StateMsgFitsInFrameBuffer) {
  StateMsg st{};
  initHeader(st, MsgType::kState, 1);
  st.joint_count = 6;
  uint8_t frame[kMaxFrameSize];
  EXPECT_GT(encodeFrame(st, frame, sizeof(frame)), 0u);
  uint8_t too_small[16];
  EXPECT_EQ(encodeFrame(st, too_small, sizeof(too_small)), 0u);
}

TEST(Framing, VariableLengthLog) {
  LogMsg log{};
  initHeader(log, MsgType::kLog, 3);
  log.level = 2;
  const char* text = "hello";
  std::memcpy(log.text, text, 5);
  const std::size_t len = sizeof(Header) + 1 + 5;

  uint8_t frame[kMaxFrameSize];
  const std::size_t n = encodeFrame(&log, len, frame, sizeof(frame));
  ASSERT_GT(n, 0u);
  FrameParser parser;
  bool got = false;
  for (std::size_t i = 0; i < n; ++i) got |= parser.feed(frame[i]);
  ASSERT_TRUE(got);
  EXPECT_TRUE(parser.isType(MsgType::kLog));
  EXPECT_FALSE(parser.is<LogMsg>(MsgType::kLog));  // not full-size
  EXPECT_EQ(parser.size(), len);
  EXPECT_EQ(std::memcmp(parser.payload() + sizeof(Header) + 1, text, 5), 0);
}
