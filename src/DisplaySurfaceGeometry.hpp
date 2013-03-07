/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#ifndef DISPLAY_SCREEN_GEOMETRY_H
#define DISPLAY_SCREEN_GEOMETRY_H
#include <iostream>

#include <osg/Geometry>

#include <jansson.h>

class GeomModel {
public:
	virtual osg::ref_ptr<osg::Geometry> make_geom(bool texcoord_colors=false) = 0;
};

class DisplaySurfaceGeometry {
public:
	DisplaySurfaceGeometry(json_t *json);
	DisplaySurfaceGeometry(const char *json);
	osg::ref_ptr<osg::Geometry> make_geom(bool texcoord_colors=false);
private:
	void parse_json(json_t *json);
	GeomModel* _geom;
};
#endif
