/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */

/* Stimulus2DSprite

   Draw a sprite (textured rectangle) on the screen.

 */

#include "vros_display/stimulus_interface.h"
#include "Poco/ClassLibrary.h"
#include "Poco/Path.h"
#include <stdexcept>
#include <sstream>

#include "base64.h"
#include <assert.h>

#include <jansson.h>

#include <osg/TextureRectangle>
#include <osg/Texture2D>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

class Stimulus2DSprite: public StimulusInterface
{
public:
Stimulus2DSprite() : anchor("center") {
	pat = new osg::PositionAttitudeTransform;
}

virtual void post_init(void) {
    std::string fname = get_plugin_data_path(std::string("cursor.png"));
    osg::ref_ptr<osg::Image> image = osgDB::readImageFile(fname);
	_load_image(image);
}

osg::ref_ptr<osg::Group> get_2d_hud() {
    return pat;
}

void _load_image(osg::Image* image) {
	pat->removeChildren(0,pat->getNumChildren()); // remove existing geode

	osg::Texture2D* texture = new osg::Texture2D(image);
	osg::Geode* geode = new osg::Geode;
	{
		osg::Geometry* geometry;
		if (anchor=="center") {
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
	pat->addChild(geode);
}

std::string name() const {
    return "Stimulus2DSprite";
}

std::vector<std::string> get_topic_names() const {
	std::vector<std::string> result;
	result.push_back( std::string("sprite_image") );
	result.push_back( std::string("sprite_pose") );
	return result;
}

std::string get_message_type(const std::string& topic_name) const {
	if (topic_name=="sprite_image") {
		return "vros_display.msg.VROSCompressedImage";
	} else if  (topic_name=="sprite_pose") {
		return "geometry_msgs.msg.Pose2D";
	}
	throw std::runtime_error("unknown topic_name");
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
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
			_load_image(image);	
			return; // success
		}
    } else {
		fprintf(stderr, "error: in %s(%d): no rw for '%s'\n", __FILE__, __LINE__,image_format.c_str());
		assert(false);
	}
	fprintf(stderr, "error: in %s(%d): bad image read\n", __FILE__, __LINE__);
	assert(false);
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
    pat->setPosition(newpos);	

	return; // success
  }
  throw std::runtime_error("unknown topic_name");
}

void resized(int width,int height) {
	_height = height;
} 

private:
	osg::ref_ptr<osg::PositionAttitudeTransform> pat;
	std::string anchor;
	int _height;
};

POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(Stimulus2DSprite)
POCO_END_MANIFEST

// optional set up and clean up functions
void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
