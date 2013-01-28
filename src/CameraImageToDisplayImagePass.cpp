/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "util.h"
#include "exrutil.h"
#include "CameraImageToDisplayImagePass.h"

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <sstream>

// Texture units
#define UNIT_LIVE 0
#define UNIT_P2C 1
#define UNIT_OUT 2

CameraImageToDisplayImagePass::CameraImageToDisplayImagePass(std::string shader_dir,
                                                             osg::ref_ptr<osg::Texture> live_camera_texture,
                                                             std::string p2c_filename,
															 bool UseHDR) :
	_live_camera_texture(live_camera_texture), _UseHDR(UseHDR)
{
	double scale_width = live_camera_texture->getTextureWidth();
	double scale_height = live_camera_texture->getTextureHeight();
	osg::ref_ptr<osg::Image> image = load_exr( p2c_filename, _display_width, _display_height, scale_width, scale_height );
	_p2c_texture = new osg::TextureRectangle;
	_p2c_texture->setTextureSize( _display_width, _display_height);
	_p2c_texture->setInternalFormat(GL_RGB32F);
	_p2c_texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
	_p2c_texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);

	_p2c_texture->setImage(image);

	create_output_texture();
	_camera = new osg::Camera;
	setup_camera();
	osg::ref_ptr<osg::Group> g = create_input_geometry();
	_camera->addChild( g.get() );

	_top = new osg::Group;
	_top->addDescription("CameraImageToDisplayImagePass top node");
	_top->addChild( _camera );
	set_shader( join_path(shader_dir,"CameraImageToDisplayImagePass.vert"),
				join_path(shader_dir,"CameraImageToDisplayImagePass.frag") );

}

osg::ref_ptr<osg::Group> CameraImageToDisplayImagePass::create_input_geometry()
{
	osg::ref_ptr<osg::Group> top_group = new osg::Group;
	top_group->addDescription("CameraImageToDisplayImagePass input geometry top node");

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

	{
		float left=0.0;
		float bottom=0.0;
		float width=1.0;
		float height=1.0;
		float zpos=0.0;

		float max_tc_width= _p2c_texture->getTextureWidth();
		float max_tc_height= _p2c_texture->getTextureHeight();

		// make quad
		osg::Vec3Array* vertices = new osg::Vec3Array;
		osg::Vec2Array* tcs = new osg::Vec2Array;
		vertices->push_back(  osg::Vec3(left, bottom, zpos) ); tcs->push_back(osg::Vec2(0.0,0.0));
		vertices->push_back(  osg::Vec3(left+width, bottom, zpos) ); tcs->push_back(osg::Vec2(max_tc_width,0.0));
		vertices->push_back(  osg::Vec3(left+width, bottom+height, zpos) ); tcs->push_back(osg::Vec2(max_tc_width,max_tc_height));
		vertices->push_back(  osg::Vec3(left, bottom+height, zpos) ); tcs->push_back(osg::Vec2(0.0,max_tc_height));

		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

		osg::ref_ptr<osg::Geometry> this_geom = new osg::Geometry();
		this_geom->setVertexArray(vertices);
		this_geom->setTexCoordArray(0,tcs);
		this_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
		this_geom->setColorArray(colors.get());
		this_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
		geode->addDrawable(this_geom);
	}

	_state_set = geode->getOrCreateStateSet();
    _state_set->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    _state_set->setTextureAttributeAndModes(UNIT_LIVE, _live_camera_texture.get(), osg::StateAttribute::ON);
	_state_set->setTextureAttributeAndModes(UNIT_P2C, _p2c_texture.get(), osg::StateAttribute::ON);
	_state_set->setMode(GL_BLEND, osg::StateAttribute::ON);

    _state_set->addUniform(new osg::Uniform("liveCamera", UNIT_LIVE));
    _state_set->addUniform(new osg::Uniform("p2c", UNIT_P2C));

	top_group->addChild(geode.get());
	return top_group;
}

void CameraImageToDisplayImagePass::create_output_texture() {
	_out_texture = new osg::TextureRectangle;
	_out_texture->setDataVariance(osg::Object::DYNAMIC);
	_out_texture->setTextureSize(_display_width, _display_height);
	if (_UseHDR) {
		_out_texture->setInternalFormat(GL_RGBA32F_ARB);
		_out_texture->setSourceFormat(GL_RGBA);
		_out_texture->setSourceType(GL_FLOAT);
	} else {
		_out_texture->setInternalFormat(GL_RGBA);
	}
	_out_texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
	_out_texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
}

void CameraImageToDisplayImagePass::setup_camera()
{
    // clearing
    _camera->setClearColor(osg::Vec4(0.0f,0.0f,0.3f,0.0f)); // clear blue
    _camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // projection and view
    _camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1.0,0,1.0));
    _camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _camera->setViewMatrix(osg::Matrix::identity());

    // viewport
    _camera->setViewport(0, 0, _display_width, _display_height);

	_camera->setRenderOrder(osg::Camera::PRE_RENDER);
    _camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

    // attach the output texture
	_camera->attach(osg::Camera::COLOR_BUFFER, _out_texture.get(), 0);

}

void CameraImageToDisplayImagePass::set_shader(std::string vert_filename, std::string frag_filename)
{
    osg::ref_ptr<osg::Shader> vshader = new osg::Shader( osg::Shader::VERTEX );
	{
		std::string fqFileName = osgDB::findDataFile(vert_filename);
		if( fqFileName.length() == 0 )
			{
				std::stringstream ss;
				ss << "File \"" << vert_filename << "\" not found.";
				throw std::ios_base::failure(ss.str());
			}
		vshader->loadShaderSourceFromFile(fqFileName);
	}

    osg::ref_ptr<osg::Shader> fshader = new osg::Shader( osg::Shader::FRAGMENT );
	{
		std::string fqFileName = osgDB::findDataFile(frag_filename);
		if( fqFileName.length() == 0 )
			{
				std::stringstream ss;
				ss << "File \"" << frag_filename << "\" not found.";
				throw std::ios_base::failure(ss.str());
			}
		fshader->loadShaderSourceFromFile(fqFileName);
	}

    _program = new osg::Program;

    _program->addShader(vshader.get());
    _program->addShader(fshader.get());
	_program->addBindFragDataLocation( "MyFragColor", 0);
    _state_set->setAttributeAndModes(_program.get(), osg::StateAttribute::ON);// | osg::StateAttribute::OVERRIDE );
}
