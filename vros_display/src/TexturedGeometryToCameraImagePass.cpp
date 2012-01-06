/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#include <osg/TextureCubeMap>
#include <osg/Group>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/TextureRectangle>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>

#include <stdexcept>
#include <sstream>

#include "util.h"
#include "TexturedGeometryToCameraImagePass.h"

TexturedGeometryToCameraImagePass::TexturedGeometryToCameraImagePass(
															   osg::ref_ptr<osg::Group> textured_geometry,
															   CameraModel* camera_model) :
	_camera_model(camera_model), _textured_geometry(textured_geometry)
 {
  _top = new osg::Group;
  _top->addDescription("TexturedGeometryToCameraImagePass top group");

  create_output_texture();

  _camera = new osg::Camera;
  setup_camera();
  _camera->addChild( _textured_geometry.get() );
  _top->addChild( _camera );
}

void TexturedGeometryToCameraImagePass::create_output_texture() {
	_out_texture = new osg::TextureRectangle;
	_out_texture->setDataVariance(osg::Object::DYNAMIC);
	_out_texture->setTextureSize(_camera_model->width(), _camera_model->height());
	_out_texture->setInternalFormat(GL_RGBA);
	_out_texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
	_out_texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
}

void TexturedGeometryToCameraImagePass::setup_camera()
{
    // clearing
    _camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,1.0f)); //black
    _camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // projection and view
	float znear=0.1;
	float zfar=10.0;
	_camera->setProjectionMatrix(_camera_model->projection(znear,zfar));
    _camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	_camera->setViewMatrixAsLookAt(_camera_model->eye(),
								   _camera_model->center(),
								   _camera_model->up());

    // viewport
    _camera->setViewport(0, 0, _camera_model->width(), _camera_model->height());

    _camera->setRenderOrder(osg::Camera::PRE_RENDER);
    _camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

    // attach the output texture
	_camera->attach(osg::Camera::COLOR_BUFFER, _out_texture.get(), 0);
}
