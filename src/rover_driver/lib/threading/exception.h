#pragma once
#include <exception>

#define EXPECT_END_OF_STREAM                                                   \
  catch (threading::EOS &) {                                                   \
    /* Normal termination */                                                   \
  }

namespace threading {

class EOS : public std::exception {};

} // namespace threading
