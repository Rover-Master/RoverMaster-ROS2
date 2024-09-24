#pragma once

#include <iomanip>
#include <sstream>
#include <string>

static inline std::string fmt(double val, unsigned n, unsigned p,
                              char fill = ' ', bool sign = true) {
  std::string s = "";
  if (sign) {
    s = val >= 0 ? '+' : '-';
    val = std::abs(val);
  }
  std::stringstream ss;
  ss << s << std::fixed << std::setprecision(p) << std::setw(n + p + 1)
     << std::setfill(fill) << val;
  return ss.str();
}
