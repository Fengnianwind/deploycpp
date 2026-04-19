#pragma once

struct RemoteState {
  bool R1 = false;
  bool L1 = false;
  bool start = false;
  bool select = false;
  bool R2 = false;
  bool L2 = false;
  bool F1 = false;
  bool A = false;
  bool B = false;
  bool X = false;
  bool Y = false;
  float lx = 0.0f;
  float ly = 0.0f;
  float rx = 0.0f;
  float ry = 0.0f;
};
