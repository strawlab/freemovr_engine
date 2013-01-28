/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

#ifndef TEXTURED_GEOMETRY_TO_CAMERA_IMAGE_PASS_H
#define TEXTURED_GEOMETRY_TO_CAMERA_IMAGE_PASS_H
#include <osg/Group>
#include <osg/Camera>
#include <osg/ref_ptr>
#include <osg/TextureRectangle>

#include "camera_model.h"

class TexturedGeometryToCameraImagePass {
public:
	TexturedGeometryToCameraImagePass(osg::ref_ptr<osg::Group> textured_geometry, CameraModel* camera_model);
	osg::ref_ptr<osg::Group> get_top() { return _top; }
	osg::ref_ptr<osg::TextureRectangle> get_output_texture() {	return _out_texture; }

private:
	void create_output_texture();
	void setup_camera();

	CameraModel* _camera_model;
	osg::ref_ptr<osg::Group> _top;
	osg::Camera* _camera;
	osg::ref_ptr<osg::Group> _textured_geometry;
	osg::ref_ptr<osg::TextureRectangle> _out_texture;
};
#endif
