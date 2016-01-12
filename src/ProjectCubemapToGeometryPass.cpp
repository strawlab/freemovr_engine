/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

#include <osg/TextureCubeMap>
#include <osg/Group>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/ShapeDrawable>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osg/io_utils>

#include <math.h>
#include <iostream>

#include <stdexcept>
#include <sstream>

#include "util.h"
#include "ProjectCubemapToGeometryPass.h"
#include "InvalidBoundsCallback.h"

#include "flyvr/flyvr_assert.h"

ProjectCubemapToGeometryPass::ProjectCubemapToGeometryPass(std::string flyvr_basepath,
                               osg::TextureCubeMap* texture,
                               osg::Uniform::Callback* observer_position_cb,
                               DisplaySurfaceGeometry* geometry_parameters,
                               unsigned int tex_width,
                               unsigned int tex_height) :
  _geometry_parameters( geometry_parameters), _tex_width(tex_width), _tex_height(tex_height), _observer_position_callback(observer_position_cb)
 {
   flyvr_assert( texture!=NULL );
   flyvr_assert( geometry_parameters!=NULL );

   set_flyvr_base_path(flyvr_basepath);
   set_plugin_path(flyvr_basepath,false);

  _top = new osg::Group;
  _top->addDescription("ProjectCubemapToGeometryPass top node");
  _in_texture_cubemap = texture;

  create_output_texture();

  _camera = new osg::Camera;
  setup_camera();
  _private_geometry = create_textured_geometry();
  _camera->addChild( _private_geometry.get() );
  _top->addChild( _camera );

  _state_set = _private_geometry->getOrCreateStateSet();

  _program = set_shader( _state_set,
                         "ProjectCubemapToGeometryPass.vert",
                         "ProjectCubemapToGeometryPass.frag");
}

void ProjectCubemapToGeometryPass::create_output_texture() {
  _out_texture = new osg::Texture2D;
  _out_texture->setDataVariance(osg::Object::DYNAMIC);
  _out_texture->setTextureSize(_tex_width, _tex_height);
  _out_texture->setInternalFormat(GL_RGBA);
  _out_texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
  _out_texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
  _out_texture->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
  _out_texture->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
}

void ProjectCubemapToGeometryPass::setup_camera()
{
    // clearing
    _camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f)); // clear black
    _camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // projection and view
    _camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1,0,1));
    _camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _camera->setViewMatrix(osg::Matrix::identity());

    // viewport
    _camera->setViewport(0, 0, _tex_width+1, _tex_height);
    // off by one error in the width? removes visible seam!

  _camera->setRenderOrder(osg::Camera::PRE_RENDER);
    _camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

    // attach the output texture
  _camera->attach(osg::Camera::COLOR_BUFFER, _out_texture.get(), 0);

}

osg::ref_ptr<osg::Program> ProjectCubemapToGeometryPass::set_shader(osg::ref_ptr<osg::StateSet> state_set, std::string vert_filename, std::string frag_filename) const
{
  osg::ref_ptr<osg::Shader> vshader = new osg::Shader( osg::Shader::VERTEX );
  osg::ref_ptr<osg::Shader> fshader = new osg::Shader( osg::Shader::FRAGMENT );

  load_shader_source( vshader, vert_filename );
  load_shader_source( fshader, frag_filename );

  osg::ref_ptr<osg::Program> program = new osg::Program;

  program->addShader(vshader.get());
  program->addShader(fshader.get());

  state_set->setAttributeAndModes(program.get(), osg::StateAttribute::ON);// | osg::StateAttribute::OVERRIDE );
  return program;
}

// use shaders to generate a texture
osg::ref_ptr<osg::Group> ProjectCubemapToGeometryPass::create_textured_geometry() const
{
    osg::ref_ptr<osg::Group> top_group = new osg::Group;
  top_group->addDescription("ProjectCubemapToGeometryPass textured geometry top node");

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  osg::ref_ptr<osg::Geometry> this_geom = _geometry_parameters->make_geom();

  // Force bounding box to be undefined, since we change vertex
  // position in the shader.
  osg::Drawable::ComputeBoundingBoxCallback* no_bounds_callback =
        new InvalidBoundsCallback();
  this_geom->setComputeBoundingBoxCallback(no_bounds_callback);

  osg::ref_ptr<osg::StateSet> state_set = this_geom->getOrCreateStateSet();
  state_set->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  state_set->setTextureAttributeAndModes(0, _in_texture_cubemap.get(), osg::StateAttribute::ON);
  state_set->setMode(GL_BLEND, osg::StateAttribute::ON);

  osg::Uniform* observerViewCubeUniformSampler = new osg::Uniform(osg::Uniform::SAMPLER_CUBE,
                                  "observerViewCube" );
  observerViewCubeUniformSampler->set(0);
  state_set->addUniform(observerViewCubeUniformSampler);

  osg::Uniform* observerPositionUniform = new osg::Uniform( "ObserverPosition",
                                osg::Vec3(0.22f, 0.22f, 0.9f) );
  if (_observer_position_callback!=NULL) {
    observerPositionUniform->setUpdateCallback(_observer_position_callback);
  }
  state_set->addUniform(observerPositionUniform);

    geode->addDrawable(this_geom.get());
    top_group->addChild(geode.get());
    return top_group;
}

// show texture on geometry
osg::ref_ptr<osg::Group> ProjectCubemapToGeometryPass::get_textured_geometry()
{
    if (_public_geometry.valid()) {
      return _public_geometry;
    }

    _public_geometry = new osg::Group;
    osg::ref_ptr<osg::Group> top_group = _public_geometry;
  top_group->addDescription("ProjectCubemapToGeometryPass output textured geometry top node");

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  osg::ref_ptr<osg::Geometry> this_geom = _geometry_parameters->make_geom();

  osg::StateSet* ss = this_geom->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    ss->setTextureAttributeAndModes(0, _out_texture, osg::StateAttribute::ON);

    geode->addDrawable(this_geom.get());
    top_group->addChild(geode.get());
    return top_group;
}
