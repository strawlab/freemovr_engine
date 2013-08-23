/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
#include "InvalidBoundsCallback.h"
#include "flyvr/flyvr_assert.h"

osg::BoundingBox InvalidBoundsCallback::computeBound(const osg::Drawable&) const  {
  osg::BoundingBox bb = osg::BoundingBox(1.0, 1.0, 1.0, -1.0, -1.0, -1.0); // max < min, thus invalid
  flyvr_assert_msg( !bb.valid(), "bounding box should be invalid" );
  return bb;
}
