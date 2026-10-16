#pragma once

#include <cmath>

namespace arm {

// First-order low-pass (exponential) filter with a cutoff in Hz. Used to
// de-noise the backward-difference velocity estimate from the encoder.
class LowPass {
 public:
  void setCutoff(float hz) { cutoff_hz_ = hz > 0.0f ? hz : 0.0f; }
  float cutoff() const { return cutoff_hz_; }

  void reset(float value = 0.0f) {
    y_ = value;
    initialised_ = true;
  }

  float update(float x, float dt) {
    if (!initialised_) {
      y_ = x;
      initialised_ = true;
      return y_;
    }
    if (cutoff_hz_ <= 0.0f || dt <= 0.0f) {
      y_ = x;  // filter disabled
      return y_;
    }
    const float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz_);
    const float alpha = dt / (rc + dt);
    y_ += alpha * (x - y_);
    return y_;
  }

  float value() const { return y_; }

 private:
  float cutoff_hz_ = 0.0f;
  float y_ = 0.0f;
  bool initialised_ = false;
};

}  // namespace arm
