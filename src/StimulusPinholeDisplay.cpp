/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "flyvr/StimulusInterface.hpp"
#include "util.h"
#include "base64.h"

#include "Poco/ClassLibrary.h"
#include "Poco/Path.h"

#include <sstream>

#include <osg/TextureRectangle>
#include <osg/AutoTransform>
#include <osgText/Text>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <stdexcept>

#include <jansson.h>

#include "flyvr/flyvr_assert.h"
#include "json2osg.hpp"
#include "DisplaySurfaceGeometry.hpp"

// this function from http://stackoverflow.com/a/8098080
std::string string_format(const std::string &fmt, ...) {
    int size=100;
    std::string str;
    va_list ap;
    while (1) {
        str.resize(size);
        va_start(ap, fmt);
        int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
        va_end(ap);
        if (n > -1 && n < size) {
            str.resize(n);
            return str;
        }
        if (n > -1)
            size=n+1;
        else
            size*=2;
    }
}

osg::Node* show_point( const osg::Vec3 position, const std::string& message ){
	float characterSize=12;
	float minScale=0.0;
	float maxScale=FLT_MAX;

    std::string timesFont("fonts/arial.ttf");

    osgText::Text* text = new osgText::Text;
    text->setCharacterSize(characterSize);
    text->setText(message);
    text->setFont(timesFont);
    text->setAlignment(osgText::Text::CENTER_CENTER);

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(text);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::AutoTransform* at = new osg::AutoTransform;
    at->addChild(geode);

    at->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
    at->setAutoScaleToScreen(true);
    at->setMinimumScale(minScale);
    at->setMaximumScale(maxScale);
    at->setPosition(position);

    return at;
}

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

    _geom_group = new osg::Group;
    _group->addChild( _geom_group );

    osg::StateSet* state = _group->getOrCreateStateSet();
    state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    osg::Program* MyProgram;
    osg::Shader*  MyVertObj;
    osg::Shader*  MyFragObj;

    MyProgram = new osg::Program;
    MyProgram->setName( "MyProgram" );
    MyVertObj = new osg::Shader( osg::Shader::VERTEX );
    MyProgram->addShader( MyVertObj );
    MyFragObj = new osg::Shader( osg::Shader::FRAGMENT );
    MyProgram->addShader( MyFragObj );

    load_shader_source( MyVertObj, "PinholeDisplay.vert" );
    load_shader_source( MyFragObj, "PinholeDisplay.frag" );

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
	result.push_back( std::string("/pinhole_cal/camera_info") );
	result.push_back( std::string("/pinhole_cal/geometry_filename") );
	result.push_back( std::string("/pinhole_cal/tf") );
	return result;
}

std::string get_message_type(const std::string& topic_name) const {
    std::string result;
    if (topic_name=="/pinhole_cal/camera_info") {
        result = "sensor_msgs/CameraInfo";
    } else if (topic_name=="/pinhole_cal/geometry_filename") {
        result = "std_msgs/String";
    } else if (topic_name=="/pinhole_cal/tf") {
        result = "geometry_msgs/Transform";
    } else {
        std::ostringstream os;
        os << "Unknown topic_name: " << topic_name;
        throw std::runtime_error(os.str());
    }
    return result;
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);
    if(!root) {
        throw std::runtime_error(string_format(
          "error: in %s(%d) on json line %d: %s\n", __FILE__, __LINE__,
          error.line, error.text));
    }

    if (topic_name=="/pinhole_cal/camera_info") {
        load_intrinsic_camera_parameters( root );
    } else if (topic_name=="/pinhole_cal/geometry_filename") {
        std::string geometry_filename = parse_string(root);
        load_geometry_filename( geometry_filename );
    } else if (topic_name=="/pinhole_cal/tf") {
        load_extrinsic_camera_parameters( root );
    } else {
        std::ostringstream os;
        os << "Unknown topic_name: " << topic_name;
        throw std::runtime_error(os.str());
    }
}

void load_intrinsic_camera_parameters( json_t * root) {
    json_t *data_json;

    data_json = json_object_get(root, "distortion_model");
    flyvr_assert(json_is_string(data_json));
    std::string distortion_model = json_string_value(data_json);
    flyvr_assert( distortion_model=="plumb_bob");

    data_json = json_object_get(root, "D");
    std::vector<double> distortion_coeffs = parse_vector_double(data_json);
    flyvr_assert( distortion_coeffs.size() == 5 );

    data_json = json_object_get(root, "K");
    std::vector<double> intrinsic_coeffs = parse_vector_double(data_json);
    flyvr_assert( intrinsic_coeffs.size() == 9 );

    // FIXME: not done - need to set projection matrix and nonlinear warping

}

void load_extrinsic_camera_parameters( json_t * root) {
    json_t *data_json;

    data_json = json_object_get(root, "translation");
    osg::Vec3 translation = parse_vec3(data_json);

    data_json = json_object_get(root, "rotation");
    osg::Quat rotation = parse_quat(data_json);

    // FIXME: not done - need to set modelview matrix

}

void load_geometry_filename( std::string geometry_filename ) {
    Poco::Path path = Poco::Path(geometry_filename);

    if (path.getExtension() == std::string("json")) {
        load_geometry_json( geometry_filename );
    } else if (path.getExtension() == std::string("osg")) {
        load_geometry_osg( geometry_filename );
    } else {
        std::ostringstream os;
        os << "Unsupported geometry_filename: " << geometry_filename;
        throw std::runtime_error(os.str());
    }
}

void load_geometry_osg( std::string geometry_filename) {
    osg::Node* node = osgDB::readNodeFile(geometry_filename);
    if (node!=NULL) {
        _group->removeChild( _geom_group );
        _geom_group = new osg::Group;
        _geom_group->addChild( node );
        _group->addChild( _geom_group );
    } else {
        std::ostringstream os;
        os << "OSG could not read node file: " << geometry_filename;
        throw std::runtime_error(os.str());
    }
}

void load_geometry_json( std::string geometry_filename ) {
    json_error_t error;
    json_t *json_geom;

    json_geom = json_load_file(geometry_filename.c_str(), 0, &error);
    if(!json_geom) {
        throw std::runtime_error(string_format(
          "error: in %s(%d) on json line %d: %s\n", __FILE__, __LINE__,
          error.line, error.text));
    }

    DisplaySurfaceGeometry* geom = new DisplaySurfaceGeometry( json_geom );
    KeyPointMap kpm = geom->get_key_points();
    _group->removeChild( _geom_group );

    _geom_group = new osg::Group;
    for (KeyPointMap::const_iterator ki=kpm.begin(); ki!=kpm.end(); ++ki) {
        _geom_group->addChild( show_point( ki->second, ki->first ) );
    }
    _group->addChild( _geom_group );

}


private:
  osg::ref_ptr<osg::Group> _group;
  osg::ref_ptr<osg::Group> _geom_group;
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
