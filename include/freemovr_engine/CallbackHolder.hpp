/* -*- Mode: C++ -*- */
#ifndef CallbackHolder_INCLUDED
#define CallbackHolder_INCLUDED

#include <osg/Referenced>
#include <osg/Vec4>

namespace freemovr_engine{

struct BackgroundColorCallback : public osg::Referenced
{
  virtual void setBackgroundColorImplementation( const osg::Vec4& ) const = 0;
};

}
#endif // CallbackHolder.hpp
