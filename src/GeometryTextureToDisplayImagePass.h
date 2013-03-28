/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

#ifndef GEOMETRY_TEXTURE_TO_DISPLAY_IMAGE_PASS_H
#define GEOMETRY_TEXTURE_TO_DISPLAY_IMAGE_PASS_H
#include <osg/Group>
#include <osg/Camera>
#include <osg/ref_ptr>
#include <osg/TextureRectangle>
#include <osg/Texture2D>

#include "Poco/Path.h"

class GeometryTextureToDisplayImagePass {
public:
	GeometryTextureToDisplayImagePass(Poco::Path shader_path,
									  osg::ref_ptr<osg::Texture2D> input_texture,
									  std::string p2g_filename,
									  bool show_geom_coords=false,
									  float display_gamma=1.0,
                                      bool red_max=false);
									  
	osg::ref_ptr<osg::Group> get_top() { return _top; }
	osg::ref_ptr<osg::Texture2D> get_output_texture() { return _out_texture; }
	int get_display_width() {return _display_width; }
	int get_display_height() {return _display_height; }
    void set_gamma(float g);
    void set_red_max(bool r);
	
private:
	void create_output_texture();
	void setup_camera();
	void set_shader(std::string vert_filename, std::string frag_filename);
	osg::ref_ptr<osg::Group> create_input_geometry();

	osg::ref_ptr<osg::Group> _top;
	osg::Camera* _camera;
	osg::ref_ptr<osg::Texture2D> _input_texture;
	osg::ref_ptr<osg::Texture2D> _p2g_texture;
	osg::ref_ptr<osg::Texture2D> _out_texture;
	osg::ref_ptr<osg::Program> _program;
    osg::ref_ptr<osg::StateSet> _state_set;
	int _display_width;
	int _display_height;
	bool _show_geom_coords;
	float _display_gamma;
    osg::ref_ptr<osg::Uniform> _display_gamma_uniform;
	bool _red_max;
    osg::ref_ptr<osg::Uniform> _red_max_uniform;
};

#endif
