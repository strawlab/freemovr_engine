/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#ifndef DISPLAY_SCREEN_GEOMETRY_H
#define DISPLAY_SCREEN_GEOMETRY_H
#include <iostream>

#include <osg/Geometry>

typedef std::map<std::string, osg::Vec3> KeyPointMap;

class GeomModel {
public:
	virtual osg::ref_ptr<osg::Geometry> make_geom(bool texcoord_colors=false) = 0;
	virtual KeyPointMap get_key_points() = 0;
};

class DisplaySurfaceGeometry {
public:
	DisplaySurfaceGeometry(std::string json_filename);
	osg::ref_ptr<osg::Geometry> make_geom(bool texcoord_colors=false);
	KeyPointMap get_key_points();
private:
	GeomModel* _geom;
};
#endif
