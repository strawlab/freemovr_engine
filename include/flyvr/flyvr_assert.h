/* -*- Mode: C++ -*- */
#ifndef flyvr_assert_INCLUDED
#define flyvr_assert_INCLUDED

#include <sstream>
#include <stdexcept>

#ifdef NDEBUG

// debugging is disabled
#define flyvr_assert_msg(OK,msg)((void)0)
#define flyvr_assert(OK)((void)0)

#else

#define flyvr_assert_msg(OK,msg) if (!(OK)) {                            \
    std::ostringstream os;                                              \
    os << "Assertion error (" << std::string(#OK) << ") at " << __FILE__ << "(" << __LINE__ << "): "<< (msg); \
    throw std::runtime_error(os.str());                                 \
  }

#define flyvr_assert(OK) if (!(OK)) {                                    \
    std::ostringstream os;                                              \
    os << "Assertion error (" << std::string(#OK) << ") at " << __FILE__ << "(" << __LINE__ << ")."; \
    throw std::runtime_error(os.str());                                 \
  }

#endif

#endif // flyvr_assert_INCLUDED
