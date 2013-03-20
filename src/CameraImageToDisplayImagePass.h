/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

#ifndef CAMERA_IMAGE_TO_DISPLAY_IMAGE_PASS_H
#define CAMERA_IMAGE_TO_DISPLAY_IMAGE_PASS_H
#include <osg/Group>
#include <osg/Camera>
#include <osg/ref_ptr>
#include <osg/TextureRectangle>
#include <osg/Texture2D>

class CameraImageToDisplayImagePass {
public:
	CameraImageToDisplayImagePass(std::string shader_dir,
								  osg::ref_ptr<osg::Texture> live_camera_texture,
								  std::string p2c_filename,
								  bool UseHDR=false);
	osg::ref_ptr<osg::Group> get_top() { return _top; }
	osg::ref_ptr<osg::Texture2D> get_output_texture() {	return _out_texture; }
	int get_display_width() {return _display_width; }
	int get_display_height() {return _display_height; }
    osg::ref_ptr<osg::StateSet> get_stateset() { return _state_set; }

private:
	void create_output_texture();
	void setup_camera();
	void set_shader(std::string vert_filename, std::string frag_filename);
	osg::ref_ptr<osg::Group> create_input_geometry();

	osg::ref_ptr<osg::Group> _top;
	osg::Camera* _camera;
	osg::ref_ptr<osg::Texture> _live_camera_texture;
	osg::ref_ptr<osg::Texture2D> _p2c_texture;
	osg::ref_ptr<osg::Texture2D> _out_texture;
	osg::ref_ptr<osg::Program> _program;
    osg::ref_ptr<osg::StateSet> _state_set;
	int _display_width;
	int _display_height;
	bool _UseHDR;

};
#endif
