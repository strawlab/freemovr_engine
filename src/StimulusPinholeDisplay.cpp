/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "flyvr/StimulusInterface.hpp"
#include "util.h"
#include "base64.h"

#include "Poco/ClassLibrary.h"

#include <sstream>

#include <osg/TextureRectangle>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <stdexcept>

#include <jansson.h>

#include "flyvr/flyvr_assert.h"

osg::ref_ptr<osg::Geometry> create_HUD_geom(unsigned int width, unsigned int height) {
	osg::ref_ptr<osg::Geometry> this_geom = new osg::Geometry();

	// make full screen textured quad

	// TODO3: Drop this and use osg::createTexturedQuadGeometry() instead.
	osg::Vec3Array* vertices = new osg::Vec3Array;
	osg::Vec2Array* texcoords = new osg::Vec2Array;

	// fullscreen quad
	vertices->push_back(  osg::Vec3(0.0,  0.0, -1.0 )); texcoords->push_back(osg::Vec2(0.0, 0.0));
	vertices->push_back(  osg::Vec3(0.0, height, -1.0 )); texcoords->push_back(osg::Vec2(0.0, height));
	vertices->push_back( osg::Vec3(width, height, -1.0 )); texcoords->push_back(osg::Vec2(width, height));
	vertices->push_back( osg::Vec3(width,  0.0, -1.0 )); texcoords->push_back(osg::Vec2(width, 0.0));

	this_geom->setVertexArray(vertices);
	this_geom->setTexCoordArray(0,texcoords);

	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
	this_geom->setColorArray(colors);
	this_geom->setColorBinding(osg::Geometry::BIND_OVERALL);

	this_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

	return this_geom;
}

class StimulusPinholeDisplay: public StimulusInterface
{
public:
	StimulusPinholeDisplay() {
	}

std::string name() const {
  return "StimulusPinholeDisplay";
}

void create_HUD_group(unsigned int width, unsigned int height, osg::Texture* texture ) {
	_group = new osg::Group;
	_group->addDescription("textured quad group");

	_geode = new osg::Geode();
	{

		// Pixel-based texcoords used here require use of TextureRectangle
		// (not Texture2D).
		osg::ref_ptr<osg::Geometry> this_geom = create_HUD_geom(width, height);
		_geode->addDrawable(this_geom);
	}
	_group->addChild(_geode);
	osg::StateSet* ss = _group->getOrCreateStateSet();
	ss->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
	ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

}

virtual void post_init(void) {
    osg::ref_ptr<osg::Texture> texture = new osg::TextureRectangle();
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR );
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);

	create_HUD_group(512, 512, texture); // Initial guess for width and height will be fixed upon call to resize().

    osg::StateSet* state = _group->getOrCreateStateSet();
    state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    osg::Program* MyProgram;
    osg::Shader*  MyVertObj;

    MyProgram = new osg::Program;
    MyProgram->setName( "MyProgram" );
    MyVertObj = new osg::Shader( osg::Shader::VERTEX );
    MyProgram->addShader( MyVertObj );

    load_shader_source( MyVertObj, "PinholeDisplay.vert" );

    state->setAttributeAndModes(MyProgram, osg::StateAttribute::ON);

    modelview = new osg::Uniform( osg::Uniform::FLOAT_MAT4, "modelview" );
	modelview->set( osg::Matrix::identity() );
    state->addUniform( modelview );

    projection = new osg::Uniform( osg::Uniform::FLOAT_MAT4, "projection" );
	//projection->set()
	projection->set( osg::Matrix::identity() );
    state->addUniform( projection );
  }

void resized(int width,int height) {
	_geode->removeDrawables(0,1);
	osg::ref_ptr<osg::Geometry> this_geom = create_HUD_geom(width, height);
	_geode->addDrawable(this_geom);
}

osg::ref_ptr<osg::Group> get_3d_world() {
    return 0;
}

osg::ref_ptr<osg::Group> get_2d_hud() {
    return _group;
}

std::vector<std::string> get_topic_names() const {
	std::vector<std::string> result;
	result.push_back( std::string("blit_images") );
	return result;
}

std::string get_message_type(const std::string& topic_name) const {
	flyvr_assert(topic_name=="blit_images");
	return "flyvr.msg.FlyVRCompressedImage";
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
	flyvr_assert(topic_name=="blit_images");

    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);

    if(!root) {
		fprintf(stderr, "error: in %s(%d) on json line %d: %s\n", __FILE__, __LINE__, error.line, error.text);
		throw std::runtime_error("error in json file");
    }

    json_t *image_data_base64_json = json_object_get(root, "data (base64)");
    if(!json_is_string(image_data_base64_json)){
		fprintf(stderr, "error: in %s(%d): expected string\n", __FILE__, __LINE__);
		throw std::runtime_error("error in json file");
    }

    json_t *image_format_json = json_object_get(root, "format");
    if(!json_is_string(image_format_json)){
		fprintf(stderr, "error: in %s(%d): expected string\n", __FILE__, __LINE__);
		throw std::runtime_error("error in json file");
    }

    std::string image_data_base64( json_string_value( image_data_base64_json ) );
    std::string image_format( json_string_value( image_format_json ));

    std::string image_data = base64_decode( image_data_base64 );

    json_decref(root);
    std::istringstream iss(image_data);

    osg::ref_ptr<osg::Image> image;
	if (image_format.at(0)=='.') {
		image_format = image_format.substr(1);
	}
    osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(image_format);
    if (rw) {
		osgDB::ReaderWriter::ReadResult rr = rw->readImage(iss);
		if ( rr.success() ) {
			image = rr.takeImage();
			osg::ref_ptr<osg::Texture> texture = new osg::TextureRectangle(image);
			osg::ref_ptr<osg::StateSet> ss = _group->getOrCreateStateSet();
			ss->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
			return; // success
		}
    } else {
		fprintf(stderr, "error: in %s(%d): no rw for '%s'\n", __FILE__, __LINE__,image_format.c_str());
		flyvr_assert(false);
	}
	fprintf(stderr, "error: in %s(%d): bad image read\n", __FILE__, __LINE__);
	flyvr_assert(false);
}

private:
  osg::ref_ptr<osg::Group> _group;
  osg::ref_ptr<osg::Geode> _geode;
  osg::Uniform* modelview, *projection;
};


POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusPinholeDisplay)
POCO_END_MANIFEST

// optional set up and clean up functions
void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
