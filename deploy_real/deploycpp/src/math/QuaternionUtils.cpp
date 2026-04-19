#include "math/QuaternionUtils.h"

#include <cmath>
#include <stdexcept>

Quat quatMul(const Quat& q1, const Quat& q2) {
  const float w1 = q1[0];
  const float x1 = q1[1];
  const float y1 = q1[2];
  const float z1 = q1[3];
  const float w2 = q2[0];
  const float x2 = q2[1];
  const float y2 = q2[2];
  const float z2 = q2[3];

  const float ww = (z1 + x1) * (x2 + y2);
  const float yy = (w1 - y1) * (w2 + z2);
  const float zz = (w1 + y1) * (w2 - z2);
  const float xx = ww + yy + zz;
  const float qq = 0.5f * (xx + (z1 - x1) * (x2 - y2));
  const float w = qq - ww + (z1 - y1) * (y2 - z2);
  const float x = qq - xx + (x1 + w1) * (x2 + w2);
  const float y = qq - yy + (w1 - x1) * (y2 + z2);
  const float z = qq - zz + (z1 + y1) * (w2 - x2);
  return {w, x, y, z};
}

Mat3 matrixFromQuat(const Quat& q) {
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];
  return {{
      {1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - z * w), 2.0f * (x * z + y * w)},
      {2.0f * (x * y + z * w), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - x * w)},
      {2.0f * (x * z - y * w), 2.0f * (y * z + x * w), 1.0f - 2.0f * (x * x + y * y)},
  }};
}

Quat yawQuat(const Quat& q) {
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];
  const float yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
  return {std::cos(yaw * 0.5f), 0.0f, 0.0f, std::sin(yaw * 0.5f)};
}

Quat eulerSingleAxisToQuat(float angle, char axis) {
  const float half_angle = angle * 0.5f;
  const float cos_half = std::cos(half_angle);
  const float sin_half = std::sin(half_angle);
  switch (axis) {
    case 'x':
    case 'X':
      return {cos_half, sin_half, 0.0f, 0.0f};
    case 'y':
    case 'Y':
      return {cos_half, 0.0f, sin_half, 0.0f};
    case 'z':
    case 'Z':
      return {cos_half, 0.0f, 0.0f, sin_half};
    default:
      throw std::runtime_error("axis must be x, y, or z");
  }
}
