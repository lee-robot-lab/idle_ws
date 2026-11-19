#pragma once

#include <cstddef>

namespace can_interface
{

class TxTickBudget
{
public:
  explicit TxTickBudget(const std::size_t max_frames_per_tick)
  : max_frames_per_tick_(max_frames_per_tick)
  {
  }

  void reset()
  {
    used_this_tick_ = 0U;
  }

  bool try_consume()
  {
    if (max_frames_per_tick_ == 0U || used_this_tick_ >= max_frames_per_tick_) {
      return false;
    }
    ++used_this_tick_;
    return true;
  }

private:
  std::size_t max_frames_per_tick_;
  std::size_t used_this_tick_ {0U};
};

}  // namespace can_interface
