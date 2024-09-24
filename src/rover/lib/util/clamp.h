#pragma once

template <typename T> T clamp(T val, T min, T max) {
  if (val < min)
    return min;
  else if (val > max)
    return max;
  else
    return val;
}
