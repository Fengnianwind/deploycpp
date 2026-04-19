#pragma once

#include <cstddef>
#include <cstdint>

#include "types/RemoteState.h"

RemoteState decodeRemoteState(const uint8_t* data, std::size_t size);
