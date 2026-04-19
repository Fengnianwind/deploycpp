#pragma once

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include "types/PolicyOutput.h"

unsigned int crc32Core(unsigned int* ptr, unsigned int len);
void buildLowCommand(
    unitree_hg::msg::dds_::LowCmd_& cmd,
    const unitree_hg::msg::dds_::LowState_& state,
    const PolicyOutput& output);
