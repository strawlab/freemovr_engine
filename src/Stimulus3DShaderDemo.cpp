/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "flyvr/StimulusInterface.hpp"

#include "Poco/ClassLibrary.h"
#include "Poco/SharedMemory.h"
#include "Poco/NamedMutex.h"

#include <iostream>

#include <osg/MatrixTransform>
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

class Stimulus3DShaderDemo: public StimulusInterface
{
public:
    Stimulus3DShaderDemo();

    std::string name() const { return "Stimulus3DShaderDemo"; }
    void post_init(bool);

    osg::ref_ptr<osg::Group> get_3d_world() {return _group; }

    std::vector<std::string> get_topic_names() const;
    void receive_json_message(const std::string& topic_name, const std::string& json_message);
    std::string get_message_type(const std::string& topic_name) const;

    void update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation );

private:
    osg::ref_ptr<osg::Group> _group;
    float _example_param;
    osg::Uniform* example_param_uniform;
    bool _slave;
    Poco::SharedMemory _mem;
    Poco::NamedMutex _memlock;
};


Stimulus3DShaderDemo::Stimulus3DShaderDemo() :
    _example_param(0.5),
    _mem("_memStimulus3DDemo", 1024, Poco::SharedMemory::AccessMode(Poco::SharedMemory::AM_WRITE | Poco::SharedMemory::AM_READ)),
    _memlock("_mutexStimulus3DDemo")  // different names for shared memory and mutex are required in windows!
{
}

void Stimulus3DShaderDemo::post_init(bool slave) {
    osg::ref_ptr<osg::Node> drawn_geometry_node = load_osg_file("Stimulus3DShaderDemo.osg");

    osg::StateSet* state = drawn_geometry_node->getOrCreateStateSet();
    state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    osg::Program* AltitudeProgram;
    osg::Shader*  AltitudeVertObj;
    osg::Shader*  AltitudeFragObj;

    AltitudeProgram = new osg::Program;
    AltitudeProgram->setName( "altitude" );
    AltitudeVertObj = new osg::Shader( osg::Shader::VERTEX );
    AltitudeFragObj = new osg::Shader( osg::Shader::FRAGMENT );
    AltitudeProgram->addShader( AltitudeFragObj );
    AltitudeProgram->addShader( AltitudeVertObj );

    load_shader_source( AltitudeVertObj, "rainbow.vert" );
    load_shader_source( AltitudeFragObj, "rainbow.frag" );

    state->setAttributeAndModes(AltitudeProgram, osg::StateAttribute::ON);
    example_param_uniform = new osg::Uniform( osg::Uniform::FLOAT, "example_param" );
    state->addUniform( example_param_uniform );

    _group = new osg::Group;
    _group->addChild(drawn_geometry_node);
    _group->setName("Stimulus3DShaderDemo._group");

    _slave = slave;
    {    
      Poco::NamedMutex::ScopedLock lock(_memlock);
      *_mem.begin() = 'a';
    }
}

std::vector<std::string> Stimulus3DShaderDemo::get_topic_names() const {
    std::vector<std::string> result;
    result.push_back("example_param");
    return result;
}

void Stimulus3DShaderDemo::receive_json_message(const std::string& topic_name,
                                                const std::string& json_message) {
    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);
    if(!root) {
        throw std::runtime_error("error in json");
    }

    json_t *data_json = json_object_get(root, "data");
    if (data_json==NULL) {
        throw std::runtime_error("key not in JSON");
    }
    if(!json_is_number(data_json)){
        throw std::runtime_error("error in json");
    }
    _example_param = json_number_value( data_json );
    example_param_uniform->set(_example_param);

}

std::string Stimulus3DShaderDemo::get_message_type(const std::string& topic_name) const {
    std::string result;

    if (topic_name=="example_param") {
        result = "std_msgs/Float32";
    } else {
        throw std::runtime_error("unknown topic name");
    }
    return result;
}

void Stimulus3DShaderDemo::update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation )
{
    Poco::NamedMutex::ScopedLock lock(_memlock);
    if (!_slave) {
        char old = *_mem.begin();
      *_mem.begin() = (old % 'z') == 0 ? 'a' : old + 1;
    }
    //std::cerr << *_mem.begin() << "\n";
}

POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(Stimulus3DShaderDemo)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
