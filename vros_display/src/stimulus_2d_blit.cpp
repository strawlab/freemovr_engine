/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#include "stimulus_interface.h"
#include "util.h"
#include "base64.h"

#include "Poco/ClassLibrary.h"

#include <sstream>
#include <assert.h>

#include <osg/TextureRectangle>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <jansson.h>

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

class Stimulus2DBlit: public StimulusInterface
{
public:
Stimulus2DBlit() {
}

std::string name() const {
  return "Stimulus2DBlit";
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

virtual void post_init(std::string config_data_dir, std::string shader_dir) {
    // TODO: show with 1:1 pixel scaling.

    std::string fname = config_data_dir + std::string("/brightday1_cubemap/posz.png");
    osg::ref_ptr<osg::Image> image = osgDB::readImageFile(fname);
    osg::ref_ptr<osg::Texture> texture = new osg::TextureRectangle(image);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR );
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);

	create_HUD_group(512, 512, texture); // Initial guess for width and height will be fixed upon call to resize().
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

std::vector<std::string> get_topic_names() {
	std::vector<std::string> result;
	result.push_back( std::string("blit_images") );
	return result;
}

std::string get_message_type(std::string topic_name) {
	assert(topic_name=="blit_images");
	return "vros_display.msg.VROSCompressedImage";
}

void send_json_message(std::string topic_name, std::string json_message) {
	assert(topic_name=="blit_images");

    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);

    if(!root) {
		fprintf(stderr, "error: in %s(%d) on json line %d: %s\n", __FILE__, __LINE__, error.line, error.text);
		throw 0;
    }

    json_t *image_data_base64_json = json_object_get(root, "data (base64)");
    if(!json_is_string(image_data_base64_json)){
		fprintf(stderr, "error: in %s(%d): expected string\n", __FILE__, __LINE__);
		throw 0;
    }

    json_t *image_format_json = json_object_get(root, "format");
    if(!json_is_string(image_format_json)){
		fprintf(stderr, "error: in %s(%d): expected string\n", __FILE__, __LINE__);
		throw 0;
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
		assert(false);
	}
	fprintf(stderr, "error: in %s(%d): bad image read\n", __FILE__, __LINE__);
	assert(false);
}

private:
  osg::ref_ptr<osg::Group> _group;
  osg::ref_ptr<osg::Geode> _geode;
};


POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(Stimulus2DBlit)
POCO_END_MANIFEST

// optional set up and clean up functions
void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
