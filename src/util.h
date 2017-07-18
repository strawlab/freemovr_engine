/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#ifndef FREEMOVR_UTIL_H
#define FREEMOVR_UTIL_H

#include <string>
#include <osg/Shader>
#include <osg/Camera>
#include <osg/Texture>
#include <osg/Group>

osg::Camera* createHUD();
osg::Group* make_textured_quad(osg::Texture* texture,
							   float zpos=-1.0,
							   float max_tc_width=1.0, float max_tc_height=1.0,
							   float left=0.0, float bottom=0.0,
							   float width=1.0, float height=1.0);

#endif
