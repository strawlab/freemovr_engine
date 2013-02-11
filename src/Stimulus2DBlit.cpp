/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "flyvr/StimulusInterface.hpp"
#include "util.h"
#include "json2osg.hpp"
#include "base64.h"

#include "Poco/ClassLibrary.h"

#include <sstream>

#include <osg/TextureRectangle>
#include <osg/Texture2D>

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

class Stimulus2DBlit: public StimulusInterface
{
public:
Stimulus2DBlit() : sprite_anchor("center") {
	rootg = new osg::Group;
    sprite_pat = new osg::PositionAttitudeTransform;
    rootg->addChild(sprite_pat);
}

std::string name() const {
  return "Stimulus2DBlit";
}

void create_HUD_group(unsigned int width, unsigned int height, osg::Texture* texture ) {
	_group = new osg::Group;
    rootg->addChild(_group);

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
    // TODO: show with 1:1 pixel scaling.

    std::string fname = get_plugin_data_path(std::string("vienna-morning.jpg"));

    {
        osg::ref_ptr<osg::Image> image = osgDB::readImageFile(fname);
        osg::ref_ptr<osg::Texture> texture = new osg::TextureRectangle(image);
        texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR );
        texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);

        create_HUD_group(512, 512, texture); // Initial guess for width and height will be fixed upon call to resize().
    }

    {
        std::string fname = get_plugin_data_path(std::string("cursor.png"));
        osg::ref_ptr<osg::Image> image = osgDB::readImageFile(fname);
        _load_sprite_image(image);
    }

  }

void _load_sprite_image(osg::Image* image) {
    sprite_pat->removeChildren(0,sprite_pat->getNumChildren()); // remove existing geode

    osg::Texture2D* texture = new osg::Texture2D(image);
    osg::Geode* geode = new osg::Geode;
    {
        osg::Geometry* geometry;
        if (sprite_anchor=="center") {
            osg::Vec3 pos = osg::Vec3( -image->s()*0.5, -image->t()*0.5, 0.0f);
            osg::Vec3 width(image->s(), 0.0, 0.0);
            osg::Vec3 height(0.0, image->t(), 0.0);
            geometry = osg::createTexturedQuadGeometry(pos,width,height);
        } else {
            throw std::runtime_error("not implemented: anchor other than center");
        }
        geode->addDrawable(geometry);

        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
        stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
        stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    }
    sprite_pat->addChild(geode);

}


void resized(int width,int height) {
    _height = height;

	_geode->removeDrawables(0,1);
	osg::ref_ptr<osg::Geometry> this_geom = create_HUD_geom(width, height);
	_geode->addDrawable(this_geom);
}

osg::ref_ptr<osg::Group> get_3d_world() {
    return 0;
}

osg::ref_ptr<osg::Group> get_2d_hud() {
    return rootg;
}

std::vector<std::string> get_topic_names() const {
	std::vector<std::string> result;
	result.push_back( std::string("blit_images") );
    result.push_back( std::string("sprite_image") );
    result.push_back( std::string("sprite_pose") );
	return result;
}

std::string get_message_type(const std::string& topic_name) const {
    if (topic_name=="blit_images") {
        return "flyvr.msg.FlyVRCompressedImage";
    } else if (topic_name=="sprite_image") {
        return "flyvr.msg.FlyVRCompressedImage";
    } else if  (topic_name=="sprite_pose") {
        return "geometry_msgs.msg.Pose2D";
    }
    throw std::runtime_error("unknown topic_name");
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
  if (topic_name=="blit_images") {
    std::string image_format;

    std::string image_data;
    parse_json_image(json_message,
                     image_format,
                     image_data);

    std::istringstream iss(image_data);

    osg::ref_ptr<osg::Image> image;
	if (image_format.at(0)=='.') {
		image_format = image_format.substr(1);
	}
    osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(image_format);
    flyvr_assert_msg( rw!=NULL, "no ReaderWriter for image_format" );
    osgDB::ReaderWriter::ReadResult rr = rw->readImage(iss);
    flyvr_assert_msg( rr.success(), "bad image read");
    image = rr.takeImage();
    osg::ref_ptr<osg::Texture> texture = new osg::TextureRectangle(image);
    osg::ref_ptr<osg::StateSet> ss = _group->getOrCreateStateSet();
    ss->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
 } else {
  json_t *root;
  json_error_t error;

  root = json_loads(json_message.c_str(), 0, &error);

  if(!root) {
      fprintf(stderr, "error: in %s(%d) on json line %d: %s\n", __FILE__, __LINE__, error.line, error.text);
      throw std::runtime_error("error in json file");
  }

  if (topic_name=="sprite_image") {

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
            _load_sprite_image(image);
            return; // success
        }
    } else {
        fprintf(stderr, "error: in %s(%d): no rw for '%s'\n", __FILE__, __LINE__,image_format.c_str());
        flyvr_assert(false);
    }
    fprintf(stderr, "error: in %s(%d): bad image read\n", __FILE__, __LINE__);
    flyvr_assert(false);
  } else if  (topic_name=="sprite_pose") {

    json_t *data_json;

    data_json = json_object_get(root, "x");
    if(!json_is_number(data_json)){
        fprintf(stderr, "error: in %s(%d): expected number\n", __FILE__, __LINE__);
        throw std::runtime_error("error in json");
    }
    double x = json_number_value( data_json );

    data_json = json_object_get(root, "y");
    if(!json_is_number(data_json)){
        fprintf(stderr, "error: in %s(%d): expected number\n", __FILE__, __LINE__);
        throw std::runtime_error("error in json");
    }
    double y = json_number_value( data_json );

    data_json = json_object_get(root, "theta");
    if(!json_is_number(data_json)){
        fprintf(stderr, "error: in %s(%d): expected number\n", __FILE__, __LINE__);
        throw std::runtime_error("error in json");
    }
    double theta = json_number_value( data_json );
    if (theta!=0.0) {
        throw std::runtime_error("theta != 0.0 not implemented.");
    }

    osg::Vec3 newpos = osg::Vec3( x, _height-y, 0.0); // convert to our coord system
    sprite_pat->setPosition(newpos);

    return; // success
  }
  throw std::runtime_error("unknown topic_name");
 }
}

private:
  osg::ref_ptr<osg::Group> rootg;
  osg::ref_ptr<osg::Group> _group;
  osg::ref_ptr<osg::Geode> _geode;

    osg::ref_ptr<osg::PositionAttitudeTransform> sprite_pat;
    std::string sprite_anchor;
    int _height;

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
