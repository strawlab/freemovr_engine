/* -*- Mode: C++ -*- */
#ifndef FREEMOVR_ASSERT_INCLUDED
#define FREEMOVR_ASSERT_INCLUDED

#include <sstream>
#include <stdexcept>

#ifdef NDEBUG

// debugging is disabled
#define freemovr_assert_msg(OK,msg)((void)0)
#define freemovr_assert(OK)((void)0)

#else

#define freemovr_assert_msg(OK,msg) if (!(OK)) {                            \
    std::ostringstream os;                                              \
    os << "Assertion error (" << std::string(#OK) << ") at " << __FILE__ << "(" << __LINE__ << "): "<< (msg); \
    throw std::runtime_error(os.str());                                 \
  }

#define freemovr_assert(OK) if (!(OK)) {                                    \
    std::ostringstream os;                                              \
    os << "Assertion error (" << std::string(#OK) << ") at " << __FILE__ << "(" << __LINE__ << ")."; \
    throw std::runtime_error(os.str());                                 \
  }

#endif

#endif // FREEMOVR_ASSERT_INCLUDED
