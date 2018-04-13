/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "freemovr_engine/StimulusInterface.hpp"
#include "freemovr_engine/freemovr_assert.h"

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
#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Bone>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <jansson.h>

//#define _DEBUG

using Poco::format;

class AnimController 
{
public:
    typedef std::vector<std::string> AnimationMapVector;

    AnimController():
        _model(0),
        _focus(0) {}

    bool setModel(osgAnimation::BasicAnimationManager* model) 
    {
        _model = model;
        _map.clear();
        _amv.clear();

        for (osgAnimation::AnimationList::const_iterator it = _model->getAnimationList().begin(); it != _model->getAnimationList().end(); it++)
            _map[(*it)->getName()] = *it;

        for(osgAnimation::AnimationMap::iterator it = _map.begin(); it != _map.end(); it++)
            _amv.push_back(it->first);

        return true;
    }

    bool list(std::ostream &output) 
    {
        for(osgAnimation::AnimationMap::iterator it = _map.begin(); it != _map.end(); it++)
            output << "Animation=\"" << it->first << "\"" << std::endl;
        return true;
    }

    bool play() 
    {
        if(_focus < _amv.size()) 
        {
            //std::cout << "Play " << _amv[_focus] << std::endl;
            _model->playAnimation(_map[_amv[_focus]].get());
            return true;
        }

        return false;
    }

    bool play(const std::string& name) 
    {
        for(unsigned int i = 0; i < _amv.size(); i++) if(_amv[i] == name) _focus = i;
        _model->playAnimation(_map[name].get());
        return true;
    }

    bool stop() 
    {
        if(_focus < _amv.size()) 
        {
            //std::cout << "Stop " << _amv[_focus] << std::endl;
            _model->stopAnimation(_map[_amv[_focus]].get());
            return true;
        }
        return false;
    }

    bool stop(const std::string& name) 
    {
        for(unsigned int i = 0; i < _amv.size(); i++) if(_amv[i] == name) _focus = i;
        _model->stopAnimation(_map[name].get());
        return true;
    }    

    bool next() 
    {
        _focus = (_focus + 1) % _map.size();
        //std::cout << "Current now is " << _amv[_focus] << std::endl;
        return true;
    }

    bool previous() 
    {
        _focus = (_map.size() + _focus - 1) % _map.size();
        //std::cout << "Current now is " << _amv[_focus] << std::endl;
        return true;
    }

    const std::string& getCurrentAnimationName() const 
    {
        return _amv[_focus];
    }

    const AnimationMapVector& getAnimationMap() const 
    {
        return _amv;
    }

private:
    osg::ref_ptr<osgAnimation::BasicAnimationManager> _model;
    osgAnimation::AnimationMap _map;
    AnimationMapVector _amv;
    unsigned int _focus;

};


struct AnimationManagerFinder : public osg::NodeVisitor
{
    osg::ref_ptr<osgAnimation::BasicAnimationManager> _am;
    AnimationManagerFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Node& node) {
        if (_am.valid())
            return;
        if (node.getUpdateCallback()) {
            osgAnimation::AnimationManagerBase* b = dynamic_cast<osgAnimation::AnimationManagerBase*>(node.getUpdateCallback());
            if (b) {
                _am = new osgAnimation::BasicAnimationManager(*b);
                return;
            }
        }
        traverse(node);
    }
};


class StimulusOSG: public StimulusInterface
{
public:

StimulusOSG(std::string p) :
   StimulusInterface(p),
    _bg_r(0.5), _bg_g(0.5), _bg_b(0.5), _bg_a(1.0),
    model_scale(1.0,1.0,1.0),
    model_position(0.,0.,0.) {
    ;
}

std::string name() const {
    return "StimulusOSG";
}

// Derive a class from NodeVisitor to find all MatrixNodes
class MatrixNodeFinder : public osg::NodeVisitor
{
public:
    MatrixNodeFinder( void )
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
    MatrixNodeFinder fmn;
    tmp->accept( fmn );
    osg::NodeList* nl = fmn.getNodeList();

    std::cerr << "Found " << nl->size() << " osg/MatrixTransform node(s): ";
    for (osg::NodeList::iterator it = nl->begin() ; it != nl->end(); ++it)
    {
        std::cerr << "\n >" << it->get()->getName() << "< ";
        SubmodelMap[it->get()->getName()] = it->get();
    }
    std::cerr << std::endl;

    // Find all animations
    AnimationManagerFinder finder;
    tmp->accept(finder);
    if (finder._am.valid()) {
        tmp->setUpdateCallback(finder._am.get());
        _anim.setModel(finder._am.get());
        _anim.list(std::cout);
    } else {
        osg::notify(osg::WARN) << "no osgAnimation::AnimationManagerBase found in the subgraph, no animations available" << std::endl;
    }
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
    freemovr_assert(switch_node.valid());
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
    result.push_back("osg_animation_start");
    result.push_back("osg_animation_stop");
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
    freemovr_assert(root != NULL);

    if (topic_name=="osg_filename") {
        _load_stimulus_filename( parse_string(root) );
    } else if (topic_name=="osg_animation_start") {
        _anim.play( parse_string(root) );
    } else if (topic_name=="osg_animation_stop") {
        _anim.stop( parse_string(root) );
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
    } else if (topic_name=="osg_animation_start") {
        result = "std_msgs/String";
    } else if (topic_name=="osg_animation_stop") {
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
    AnimController _anim;
};

MAKE_STIMULUS_INTERFACE_LOADER(StimulusOSG);

POCO_BEGIN_MANIFEST(StimulusInterfaceLoader)
POCO_EXPORT_CLASS(StimulusOSGLoader)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}

