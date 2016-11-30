/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "freemoovr/StimulusInterface.hpp"
#include "freemoovr/freemoovr_assert.h"

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

//#define _DEBUG

using Poco::format;

class StimulusOSG: public StimulusInterface
{
public:

StimulusOSG() :
    _bg_r(0.5), _bg_g(0.5), _bg_b(0.5), _bg_a(1.0),
    model_scale(1.0,1.0,1.0),
    model_position(0.,0.,0.) {
    ;
}

std::string name() const {
    return "StimulusOSG";
}

// Derive a class from NodeVisitor to find all MatrixNodes
class FindMatrixNodes : public osg::NodeVisitor
{
public:
    FindMatrixNodes( void )
      : osg::NodeVisitor( // Traverse all children.
                osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
        {}

    virtual void apply( osg::Node& node )
    {
        if (typeid(node) == typeid(osg::MatrixTransform))
        {
            _nodeList.push_back(&node);
        }
        // Keep traversing the rest of the scene graph.
        traverse( node );
    }

    osg::NodeList* getNodeList() { return &_nodeList; }

protected:
    osg::NodeList _nodeList;
};


void _load_stimulus_filename( std::string osg_filename ) {

    if (!top) {
        std::cerr << "top node not defined!?" << std::endl;
        return;
    }

    // empty submodel map
    SubmodelMap.clear();

    // don't show the old switching node.
    top->removeChild(switch_node);

    // (create a new switching node.
    switch_node = new osg::PositionAttitudeTransform;
    _update_pat();
    top->addChild(switch_node);

    // now load it with new contents
    osg::Node *tmp = load_osg_file(osg_filename);  // throws a runtime exception when file not found (catch in display_server.pyx)
    switch_node->addChild(tmp);

    std::cerr << "Loaded " << osg_filename << "\n";

    // Find all "MatrixNode"s
    FindMatrixNodes fmn;
    tmp->accept( fmn );
    osg::NodeList* nl = fmn.getNodeList();

    std::cerr << "Found " << nl->size() << " osg/MatrixTransform node(s): ";
    for (osg::NodeList::iterator it = nl->begin() ; it != nl->end(); ++it)
    {
        std::cerr << "\n >" << it->get()->getName() << "< ";
        SubmodelMap[it->get()->getName()] = it->get();
    }
    std::cerr << std::endl;
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
    freemoovr_assert(switch_node.valid());
    switch_node->setPosition( model_position );
    switch_node->setScale( model_scale );
    osg::Matrix m;
    m.makeRotate( model_attitude );
    top->setMatrix(m);
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
    return osg::Vec4(_bg_r, _bg_g, _bg_b, _bg_a);
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
    result.push_back("osg_filename");
    result.push_back("osg_skybox_basename");
    result.push_back("osg_model_pose");
    result.push_back("osg_submodel_pose");
    result.push_back("osg_model_scale");
    result.push_back("osg_background_color");
    return result;
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);
    freemoovr_assert(root != NULL);

    if (topic_name=="osg_filename") {
        _load_stimulus_filename( parse_string(root) );
    } else if (topic_name=="osg_background_color") {
        json_t *data_json;
        data_json = json_object_get(root, "r");
        _bg_r = json_number_value( data_json );
        data_json = json_object_get(root, "g");
        _bg_g = json_number_value( data_json );
        data_json = json_object_get(root, "b");
        _bg_b = json_number_value( data_json );
        data_json = json_object_get(root, "a");
        _bg_a = json_number_value( data_json );
    } else if (topic_name=="osg_skybox_basename") {
        _load_skybox_basename( parse_string(root) );
    } else if (topic_name=="osg_model_pose") {
        json_t *data_json;
        data_json = json_object_get(root, "position");
        model_position = parse_vec3(data_json);
        data_json = json_object_get(root, "orientation");
        model_attitude = parse_quat(data_json);
        _update_pat();
    } else if (topic_name=="osg_submodel_pose") {
        json_t *data_json, *nextroot;
        nextroot = json_object_get(root, "header");
        data_json = json_object_get(nextroot, "frame_id");
        std::string name = json_string_value(data_json);
        nextroot = json_object_get(root, "pose");
        data_json = json_object_get(nextroot, "position");
        osg::Vec3 position = parse_vec3(data_json);
        data_json = json_object_get(nextroot, "orientation"); // we interpret the quaternion as 3 Euler angles and w as scalefactor (HACK!)
        
        osg::Quat attitude = parse_quat(data_json);
#ifdef _DEBUG        
        std::cout << "osg_submodel_pose: name: " << name <<
        " pos: " << position.x() << ", " << position.y() << ", " << position.z() <<
        " euler orientation: " << attitude.x()  << ", " << attitude.y()  << ", " << attitude.z()  << 
        " scale: " << attitude.w()  << std::endl;
#endif
        try {
            osg::Node* n = SubmodelMap.at(name);

            osg::Matrix m; 
            m.makeIdentity();

            // use xyz as Euler angles
            osg::Quat q=osg::Quat(  
                attitude.x(), osg::Vec3(1.0,0.0,0.0),   // rotate around X-axis
                attitude.y(), osg::Vec3(0.0,1.0,0.0),   // rotate around Y-axis
                attitude.z(), osg::Vec3(0.0,0.0,1.0));  // rotate around Z-axis

            m.setRotate(q);
            m.setTrans(position);
            if (attitude.w() >= 0) {
                // use w as scale factor
                m.preMultScale(osg::Vec3(attitude.w(), attitude.w(), attitude.w()));
            }

            ((osg::MatrixTransform *)n)->setMatrix(m);

        } catch (std::out_of_range) {
            std::cerr << " node '" << name << "' does not exist!\n";
        }
    } else if (topic_name=="osg_model_scale") {
        model_scale = parse_vec3(root);
        _update_pat();
    } else {
        throw std::runtime_error("unknown topic name");
    }

    json_decref(root);
}

std::string get_message_type(const std::string& topic_name) const {
    std::string result;

    if (topic_name=="osg_filename") {
        result = "std_msgs/String";
    } else if (topic_name=="osg_skybox_basename") {
        result = "std_msgs/String";
    } else if (topic_name=="osg_model_pose") {
        result = "geometry_msgs/Pose";
    } else if (topic_name=="osg_model_scale") {
        result = "geometry_msgs/Vector3";
    } else if (topic_name=="osg_submodel_pose") {
        result = "geometry_msgs/PoseStamped";
    } else if (topic_name=="osg_background_color") {
        result = "std_msgs/ColorRGBA";
    } else {
        throw std::runtime_error(format("unknown topic name: %s",topic_name));
    }
    return result;

}

private:
    float _bg_r, _bg_g, _bg_b, _bg_a;

    osg::ref_ptr<osg::Group> _virtual_world;
    osg::ref_ptr<osg::MatrixTransform> top;
    osg::ref_ptr<osg::PositionAttitudeTransform> switch_node;
    osg::Vec3 model_scale;
    osg::Vec3 model_position;
    osg::Quat model_attitude;
    osg::ref_ptr<osg::MatrixTransform> skybox_node;
    std::map<std::string, osg::ref_ptr<osg::Node> > SubmodelMap;
};


POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusOSG)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}

