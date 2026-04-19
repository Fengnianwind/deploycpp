#pragma once

#include <array>

using Quat = std::array<float, 4>;
using Mat3 = std::array<std::array<float, 3>, 3>;

Quat quatMul(const Quat& q1, const Quat& q2);
Mat3 matrixFromQuat(const Quat& q);
Quat yawQuat(const Quat& q);
Quat eulerSingleAxisToQuat(float angle, char axis);
