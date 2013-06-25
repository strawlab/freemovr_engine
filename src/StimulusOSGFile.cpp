/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "flyvr/StimulusInterface.hpp"
#include "flyvr/flyvr_assert.h"

#include "json2osg.hpp"

#include "Poco/ClassLibrary.h"
#include "Poco/Format.h"

#include <iostream>

#include <osg/MatrixTransform>
#include <osg/TextureCubeMap>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/CullFace>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LightModel>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <jansson.h>

using Poco::format;

class StimulusOSGFile: public StimulusInterface
{
public:

StimulusOSGFile() : 
    model_scale(1.0,1.0,1.0),
    model_position(0.,0.,0.) {
    ;
}

std::string name() const {
    return "StimulusOSGFile";
}

void _load_stimulus_filename( std::string osg_filename ) {


    if (!top) {
        std::cerr << "top node not defined!?" << std::endl;
        return;
    }

    // don't show the old switching node.
    top->removeChild(switch_node);

    // (rely on C++ to delete the old switching node).

    // (create a new switching node.
    switch_node = new osg::PositionAttitudeTransform;
    _update_pat();

    // now load it with new contents
    std::cerr << "reading .osg file: " << osg_filename << std::endl;

    osg::Node* tmp = load_osg_file(osg_filename);
    if (tmp!=NULL) {
        switch_node->addChild( tmp );
    } else {
        throw std::runtime_error(format("File %s not found",osg_filename));
    }

    top->addChild(switch_node);
}

void _load_skybox_basename( std::string basename ) {

    if (!top) {
        std::cerr << "top node not defined!?" << std::endl;
        return;
    }

    if (skybox_node.valid()) {
        top->removeChild(skybox_node);
        skybox_node = NULL; // dereference the previous node
    }

    if (basename=="<none>") {
        return;
    }

    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;

    if (basename=="<default>") {
        add_default_skybox( mt );
    } else {
        std::string extension = ".png";
        try {
            add_skybox(mt, basename, extension);
        } catch (std::runtime_error) {
            extension = ".jpg";
            add_skybox(mt, basename, extension);
        }

    }

    skybox_node = mt;
    top->addChild(skybox_node);
}

void _update_pat() {
    flyvr_assert(switch_node.valid());
    switch_node->setPosition( model_position );
    switch_node->setScale( model_scale );
    switch_node->setAttitude( model_attitude );
}

virtual void post_init(bool slave) {
  top = new osg::MatrixTransform;
  top->addDescription("virtual world top node");

  // when we first start, don't load any model, but create a node that is later deleted.
  switch_node = new osg::PositionAttitudeTransform;
  _update_pat();
  top->addChild(switch_node);

  _virtual_world = top;
}

osg::Vec4 get_clear_color() const {
    return osg::Vec4(0.5, 0.5, 0.5, 1.0); // mid gray
}

void resized(int width,int height) {
}

osg::ref_ptr<osg::Group> get_3d_world() {
    return _virtual_world;
}

osg::ref_ptr<osg::Group> get_2d_hud() {
    return 0;
}

std::vector<std::string> get_topic_names() const {
    std::vector<std::string> result;
    result.push_back("stimulus_filename");
    result.push_back("skybox_basename");
    result.push_back("model_pose");
    result.push_back("model_scale");
    return result;
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);
    flyvr_assert(root != NULL);

    if (topic_name=="stimulus_filename") {
        _load_stimulus_filename( parse_string(root) );
    } else if (topic_name=="skybox_basename") {
        _load_skybox_basename( parse_string(root) );
    } else if (topic_name=="model_pose") {
        json_t *data_json;
        data_json = json_object_get(root, "position");
        model_position = parse_vec3(data_json);
        data_json = json_object_get(root, "orientation");
        model_attitude = parse_quat(data_json);
        _update_pat();
    } else if (topic_name=="model_scale") {
        model_scale = parse_vec3(root);
        _update_pat();
    } else {
        throw std::runtime_error("unknown topic name");
    }

    json_decref(root);
}

std::string get_message_type(const std::string& topic_name) const {
    std::string result;

    if (topic_name=="stimulus_filename") {
        result = "std_msgs/String";
    } else if (topic_name=="skybox_basename") {
        result = "std_msgs/String";
    } else if (topic_name=="model_pose") {
        result = "geometry_msgs/Pose";
    } else if (topic_name=="model_scale") {
        result = "geometry_msgs/Vector3";
    } else {
        throw std::runtime_error(format("unknown topic name: %s",topic_name));
    }
    return result;

}

private:
    osg::ref_ptr<osg::Group> _virtual_world;
    osg::ref_ptr<osg::MatrixTransform> top;
    osg::ref_ptr<osg::PositionAttitudeTransform> switch_node;
    osg::Vec3 model_position;
    osg::Vec3 model_scale;
    osg::Quat model_attitude;
    osg::ref_ptr<osg::MatrixTransform> skybox_node;
};


POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusOSGFile)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
