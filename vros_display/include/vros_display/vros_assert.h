/* -*- Mode: C++ -*- */
#ifndef vros_assert_INCLUDED
#define vros_assert_INCLUDED

#include <sstream>
#include <stdexcept>

#ifdef NDEBUG

// debugging is disabled
#define vros_assert_msg(OK,msg)((void)0)
#define vros_assert(OK)((void)0)

#else

#define vros_assert_msg(OK,msg) if (!(OK)) {                            \
    std::ostringstream os;                                              \
    os << "Assertion error (" << std::string(#OK) << ") at " << __FILE__ << "(" << __LINE__ << "): "<< (msg); \
    throw std::runtime_error(os.str());                                 \
  }

#define vros_assert(OK) if (!(OK)) {                                    \
    std::ostringstream os;                                              \
    os << "Assertion error (" << std::string(#OK) << ") at " << __FILE__ << "(" << __LINE__ << ")."; \
    throw std::runtime_error(os.str());                                 \
  }

#endif

#endif // vros_assert_INCLUDED
