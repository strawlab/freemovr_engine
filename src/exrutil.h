/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#ifndef FLYVR_EXRUTIL_H
#define FLYVR_EXRUTIL_H

#include <string>
#include <osg/Image>

void save_exr( std::string filename, osg::Image* image );
osg::ref_ptr<osg::Image> load_exr( std::string p2c_filename, int& width, int& height,
								   double scale_width=1.0, double scale_height=1.0 );

#endif
