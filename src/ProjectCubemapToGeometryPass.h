/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

#ifndef PROJECT_CUBEMAP_TO_GEOMETRY_PASS
#define PROJECT_CUBEMAP_TO_GEOMETRY_PASS
#include <osg/TextureCubeMap>
#include <osg/Group>
#include <osg/Camera>
#include <osg/ref_ptr>
#include <osg/Texture2D>

#include "DisplaySurfaceGeometry.hpp"
#include "flyvr/ResourceLoader.hpp"

class ProjectCubemapToGeometryPass: protected ResourceLoader {
public:
    ProjectCubemapToGeometryPass(std::string flyvr_basepath,
                                 osg::TextureCubeMap* texture,
                                 osg::Uniform::Callback* observer_position_cb,
                                 DisplaySurfaceGeometry* geometry_parameters,
                                 unsigned int tex_width=512, unsigned int tex_height=512);
    osg::ref_ptr<osg::Group> get_top() { return _top; }
    osg::ref_ptr<osg::Texture2D> get_output_texture() { return _out_texture; }
    osg::ref_ptr<osg::Group> get_textured_geometry();

private:
    void create_output_texture();
    void setup_camera();
    osg::ref_ptr<osg::Group> create_textured_geometry();
    osg::ref_ptr<osg::Group> create_output_textured_quad();
    void set_shader(std::string vert_filename, std::string frag_filename);

    DisplaySurfaceGeometry* _geometry_parameters;
    unsigned int _tex_width;
    unsigned int _tex_height;
    osg::ref_ptr<osg::Group> _top;
    osg::Camera* _camera;
    osg::Uniform::Callback* _observer_position_callback;
    osg::ref_ptr<osg::TextureCubeMap> _in_texture_cubemap;
    osg::ref_ptr<osg::Group> _private_geometry;
    osg::ref_ptr<osg::Group> _public_geometry;

    osg::ref_ptr<osg::Group> _output_quad;
    osg::ref_ptr<osg::Texture2D> _out_texture;
    osg::ref_ptr<osg::Program> _program;
    osg::ref_ptr<osg::StateSet> _state_set;
};
#endif
