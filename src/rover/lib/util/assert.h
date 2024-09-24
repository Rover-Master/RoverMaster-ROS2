#pragma once

#include <sstream>
#include <stdexcept>

class AssertionError : public std::runtime_error {
public:
  // Inherit the constructors from std::runtime_error
  using std::runtime_error::runtime_error;
};

#define ASSERT(COND, MESSAGE)                                                  \
  {                                                                            \
    if (!(COND)) {                                                             \
      std::stringstream ss;                                                    \
      ss << MESSAGE;                                                           \
      throw AssertionError(ss.str());                                          \
    }                                                                          \
  }

#define CATCH_ASSERT(LOGNAME)                                                  \
  catch (AssertionError & e) {                                                 \
    std::cerr << "[ASSERTION ERROR] " << e.what() << std::endl;                \
  }                                                                            \
  catch (std::exception & e) {                                                 \
    std::cerr << LOGNAME << e.what() << std::endl;                             \
  }                                                                            \
  catch (...) {                                                                \
    std::cerr << LOGNAME "Unknown Error" << std::endl;                         \
  }
