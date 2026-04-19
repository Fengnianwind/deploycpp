#pragma once

#include <utility>

template <typename Work, typename Fallback>
void runWithDampingFallback(Work&& work, Fallback&& fallback) {
  try {
    std::forward<Work>(work)();
  } catch (...) {
    try {
      std::forward<Fallback>(fallback)();
    } catch (...) {
    }
    throw;
  }
}
