/* -*- Mode: C++ -*- */
#ifndef FREEMOOVR_ASSERT_INCLUDED
#define FREEMOOVR_ASSERT_INCLUDED

#include <sstream>
#include <stdexcept>

#ifdef NDEBUG

// debugging is disabled
#define freemoovr_assert_msg(OK,msg)((void)0)
#define freemoovr_assert(OK)((void)0)

#else

#define freemoovr_assert_msg(OK,msg) if (!(OK)) {                            \
    std::ostringstream os;                                              \
    os << "Assertion error (" << std::string(#OK) << ") at " << __FILE__ << "(" << __LINE__ << "): "<< (msg); \
    throw std::runtime_error(os.str());                                 \
  }

#define freemoovr_assert(OK) if (!(OK)) {                                    \
    std::ostringstream os;                                              \
    os << "Assertion error (" << std::string(#OK) << ") at " << __FILE__ << "(" << __LINE__ << ")."; \
    throw std::runtime_error(os.str());                                 \
  }

#endif

#endif // FREEMOOVR_ASSERT_INCLUDED
