/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

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

#include "freemovr_engine/freemovr_assert.h"

ProjectCubemapToGeometryPass::ProjectCubemapToGeometryPass(std::string freemovr_basepath,
                                                             osg::TextureCubeMap* texture,
                                                             osg::Uniform::Callback* observer_position_cb,
                                                             DisplaySurfaceGeometry* geometry_parameters,
                                                             unsigned int tex_width,
                                                             unsigned int tex_height) :
    _geometry_parameters( geometry_parameters), _tex_width(tex_width), _tex_height(tex_height), _observer_position_callback(observer_position_cb)
 {
     freemovr_assert( texture!=NULL );
     freemovr_assert( geometry_parameters!=NULL );

     set_freemovr_base_path(freemovr_basepath);

    _top = new osg::Group;
    _top->addDescription("ProjectCubemapToGeometryPass top node");
    _in_texture_cubemap = texture;

    _observerPositionUniform = new osg::Uniform( "ObserverPosition",
                                                 osg::Vec3(0.22f, 0.22f, 0.9f) );
    if (_observer_position_callback!=NULL) {
      _observerPositionUniform->setUpdateCallback(_observer_position_callback);
    }

    create_output_texture();

    _camera = new osg::Camera;
    setup_camera();
    _private_geometry = create_textured_geometry();
    _camera->addChild( _private_geometry.get() );
    _top->addChild( _camera );

    _state_set = _private_geometry->getOrCreateStateSet();
    _state_set->addUniform(_observerPositionUniform);

    _program = set_shader( _state_set,
                           "ProjectCubemapToGeometryPass.vert",
                           "ProjectCubemapToGeometryPass.frag");

    _public_geometry = new osg::Group;
    _public_geometry->addDescription("ProjectCubemapToGeometryPass output textured geometry top node");

    _inner_geode = _create_textured_geometry_inner_geode();
    _public_geometry->addChild(_inner_geode.get());
}

void ProjectCubemapToGeometryPass::replace_display_surface_geometry( DisplaySurfaceGeometry* geometry_parameters ) {
    freemovr_assert( geometry_parameters!=NULL );

    // Tear down usage of previous display surface geometry.
    if (_state_set.valid()) {
      if (_observerPositionUniform != NULL) {
        _state_set->removeUniform(_observerPositionUniform);
      }
    }

    if (_private_geometry.valid()) {
      _camera->removeChild( _private_geometry.get() );
    }

    if (_inner_geode.valid()) {
      _public_geometry->removeChild(_inner_geode.get());
    }

    // Use new display surface geometry.
    _geometry_parameters = geometry_parameters;

    _private_geometry = create_textured_geometry();
    _camera->addChild( _private_geometry.get() );

    _state_set = _private_geometry->getOrCreateStateSet();
    _state_set->addUniform(_observerPositionUniform);
    _program = set_shader( _state_set,
                           "ProjectCubemapToGeometryPass.vert",
                           "ProjectCubemapToGeometryPass.frag");

    _inner_geode = _create_textured_geometry_inner_geode();
    _public_geometry->addChild(_inner_geode.get());

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

    geode->addDrawable(this_geom.get());
    top_group->addChild(geode.get());
    return top_group;
}

// show texture on geometry
osg::ref_ptr<osg::Group> ProjectCubemapToGeometryPass::get_textured_geometry() const
{
    freemovr_assert(_public_geometry.valid());
    return _public_geometry;
}

// show texture on geometry inner
osg::ref_ptr<osg::Geode> ProjectCubemapToGeometryPass::_create_textured_geometry_inner_geode() const
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> this_geom = _geometry_parameters->make_geom();

    osg::StateSet* ss = this_geom->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    ss->setTextureAttributeAndModes(0, _out_texture, osg::StateAttribute::ON);

    geode->addDrawable(this_geom.get());
    return geode;
}
