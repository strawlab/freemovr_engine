/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "flyvr/StimulusInterface.hpp"
#include "flyvr/flyvr_assert.h"

#include "json2osg.hpp"

#include "Poco/ClassLibrary.h"
#include "Poco/SharedMemory.h"
#include "Poco/NamedMutex.h"

#include <iostream>
#include <stdio.h>
#include <string.h>

#include <osg/MatrixTransform>
#include <osg/TextureCubeMap>
#include <osg/Texture2D>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/CullFace>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LightModel>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

typedef struct
{
    float angular_position;
    float v_offset_value;
} StimulusCylinderSharedStateType;

class StimulusCylinder: public StimulusInterface
{
public:
    StimulusCylinder();

    void post_init(bool);
    std::vector<std::string> get_topic_names() const;
    void receive_json_message(const std::string& topic_name, const std::string& json_message);
    std::string get_message_type(const std::string& topic_name) const;
    void update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation );

    std::string name() const { return "StimulusCylinder"; }
    osg::ref_ptr<osg::Group> get_3d_world() {return _virtual_world; }

private:
    const static float DEFAULT_RADIUS = 0.5f;
    const static float DEFAULT_HEIGHT = 1.0f;

    osg::ref_ptr<osg::Group>            _virtual_world;
    osg::ref_ptr<osg::Cylinder>         _cylinder;
    osg::ref_ptr<osg::Texture2D>        _texture;
    osg::ref_ptr<osg::ShapeDrawable>    _shape;
    double                              _t0;
    double                              _angular_position;
    double                              _angular_velocity;
    bool                                _angular_position_mode;
    double                              _v_offset_value;
    double                              _v_offset_rate;
    bool                                _v_offset_value_mode;
    bool                                _slave;
    bool                                _lock_pose_x;
    bool                                _lock_pose_y;
    bool                                _lock_pose_z;
    Poco::SharedMemory                  _mem;
    Poco::NamedMutex                    _memlock;

    osg::Uniform* tex_v_offset_uniform;

    osg::ref_ptr<osg::Group> create_virtual_world();
    void dirty_cylinder(void);
    void set_cylinder_rotation(float angle);
    void set_cylinder_rotation_rate(float rate);
    void set_cylinder_position(float x, float y, float z);
    void set_cylinder_radius(float r);
    void set_cylinder_height(float h);
    void set_cylinder_image(std::string s) { _texture->setImage(load_image_file(s)); }
    void set_cylinder_v_offset(float vo);
    void set_cylinder_v_offset_rate(float vor);
};

StimulusCylinder::StimulusCylinder() :
    _t0(-1),
    _angular_position(0),
    _angular_velocity(0),
    _angular_position_mode(true),
    _lock_pose_x(false),
    _lock_pose_y(false),
    _lock_pose_z(false),
    _v_offset_value(0),
    _v_offset_rate(0),
    _v_offset_value_mode(true),
    _mem("StimulusCylinder", sizeof(StimulusCylinderSharedStateType),
         Poco::SharedMemory::AccessMode(Poco::SharedMemory::AM_WRITE | Poco::SharedMemory::AM_READ)),
    _memlock("StimulusCylinder")
{

}

void StimulusCylinder::post_init(bool slave)
{
    _slave = slave;
    _virtual_world = create_virtual_world();

    if (!_slave) {
        Poco::NamedMutex::ScopedLock lock(_memlock);
        memset (_mem.begin(),0,sizeof(StimulusCylinderSharedStateType));
    }
}

std::vector<std::string> StimulusCylinder::get_topic_names() const
{
    std::vector<std::string> result;
    result.push_back("cylinder_radius");
    result.push_back("cylinder_height");
    result.push_back("cylinder_rotation");
    result.push_back("cylinder_rotation_rate");
    result.push_back("cylinder_v_offset_value");
    result.push_back("cylinder_v_offset_rate");
    result.push_back("cylinder_image");
    result.push_back("cylinder_centre");
    result.push_back("cylinder_lock_pose_x");
    result.push_back("cylinder_lock_pose_y");
    result.push_back("cylinder_lock_pose_z");
    return result;
}

void StimulusCylinder::receive_json_message(const std::string& topic_name, const std::string& json_message)
{
    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);
    flyvr_assert(root != NULL);

    if (topic_name=="cylinder_radius") {
        set_cylinder_radius(parse_float(root));
    } else if (topic_name=="cylinder_rotation") {
        set_cylinder_rotation(parse_float(root));
    } else if (topic_name=="cylinder_rotation_rate") {
        set_cylinder_rotation_rate(parse_float(root));
    } else if (topic_name=="cylinder_height") {
        set_cylinder_height(parse_float(root));
    } else if (topic_name=="cylinder_v_offset_value") {
        set_cylinder_v_offset(parse_float(root));
    } else if (topic_name=="cylinder_v_offset_rate") {
        set_cylinder_v_offset_rate(parse_float(root));
    } else if (topic_name=="cylinder_image") {
        set_cylinder_image(parse_string(root));
    } else if (topic_name=="cylinder_centre") {
        osg::Vec3 position = parse_vec3(root);
        set_cylinder_position(position.x(),position.y(),position.z());
    } else if (topic_name=="cylinder_lock_pose_x") {
        _lock_pose_x = parse_int(root);
    } else if (topic_name=="cylinder_lock_pose_y") {
        _lock_pose_y = parse_int(root);
    } else if (topic_name=="cylinder_lock_pose_z") {
        _lock_pose_z = parse_int(root);
    } else {
        throw std::runtime_error("unknown topic name");
    }

    json_decref(root);
}

std::string StimulusCylinder::get_message_type(const std::string& topic_name) const
{
    std::string result;

    if (topic_name=="cylinder_radius") {
        result = "std_msgs/Float32";
    } else if (topic_name=="cylinder_height") {
        result = "std_msgs/Float32";
    } else if (topic_name=="cylinder_rotation") {
        result = "std_msgs/Float32";
    } else if (topic_name=="cylinder_rotation_rate") {
        result = "std_msgs/Float32";
    } else if (topic_name=="cylinder_v_offset_value") {
        result = "std_msgs/Float32";
    } else if (topic_name=="cylinder_v_offset_rate") {
        result = "std_msgs/Float32";
    } else if (topic_name=="cylinder_image") {
        result = "std_msgs/String";
    } else if (topic_name=="cylinder_centre") {
        result = "geometry_msgs/Vector3";
    } else if (topic_name=="cylinder_lock_pose_x") {
        result = "std_msgs/UInt8";
    } else if (topic_name=="cylinder_lock_pose_y") {
        result = "std_msgs/UInt8";
    } else if (topic_name=="cylinder_lock_pose_z") {
        result = "std_msgs/UInt8";
    } else {
        throw std::runtime_error("unknown topic name");
    }
    return result;
}

void StimulusCylinder::update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation )
{
  Poco::NamedMutex::ScopedLock lock(_memlock);
  StimulusCylinderSharedStateType *shared;
  shared = reinterpret_cast<StimulusCylinderSharedStateType*>(_mem.begin());

  if (!_slave) {
    if (_t0 < 0) {
        _t0 = time;
        return;
    }
    float dt = time - _t0;
    _t0 = time;

    if (_angular_position_mode) {
        shared->angular_position = _angular_position;
    } else {
        shared->angular_position = shared->angular_position + (_angular_velocity * dt);
    }

    if (_v_offset_value_mode) {
        shared->v_offset_value = _v_offset_value;
    } else {
        shared->v_offset_value = shared->v_offset_value + (_v_offset_rate * dt);
    }

  }

  tex_v_offset_uniform->set(shared->v_offset_value);


  if (_lock_pose_x) {
      const osg::Vec3& orig_center = _cylinder->getCenter();
      set_cylinder_position(observer_position[0], orig_center[1], orig_center[2]-(0.5*DEFAULT_HEIGHT));
  }

  if (_lock_pose_y) {
      const osg::Vec3& orig_center = _cylinder->getCenter();
      set_cylinder_position(orig_center[0], observer_position[1], orig_center[2]-(0.5*DEFAULT_HEIGHT));
  }

  if (_lock_pose_z) {
      const osg::Vec3& orig_center = _cylinder->getCenter();
      set_cylinder_position(orig_center[0], orig_center[1], observer_position[2]);
  }

  osg::Quat quat = osg::Quat(shared->angular_position, osg::Vec3(0,0,1));
  _cylinder->setRotation(quat);
  dirty_cylinder();

}

osg::ref_ptr<osg::Group> StimulusCylinder::create_virtual_world() {
    osg::ref_ptr<osg::MatrixTransform> myroot = new osg::MatrixTransform; myroot->addDescription("virtual world root node");

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    myroot->addChild(geode.get());

    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
    hints->setDetailRatio(2.0f);
    hints->setCreateTop(false);
    hints->setCreateBottom(false);

    //start in absolute position mode
    _cylinder = new osg::Cylinder();
    _shape = new osg::ShapeDrawable(_cylinder, hints.get());

    set_cylinder_rotation(_angular_position);
    set_cylinder_position(0.0,0.0,0.0);
    set_cylinder_radius(DEFAULT_RADIUS);
    set_cylinder_height(DEFAULT_HEIGHT);

    geode->addDrawable(_shape.get());

    _texture = new osg::Texture2D();
    _texture->setDataVariance(osg::Object::DYNAMIC);
    _texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
    _texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
    _texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    _texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    set_cylinder_image("checkerboard16.png");

    osg::StateSet *cylStateSet = _shape->getOrCreateStateSet();
    cylStateSet->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);

    cylStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    cylStateSet->setMode(GL_BLEND, osg::StateAttribute::ON);

    {
        osg::Program* ZtexcylProgram;
        osg::Shader*  ZtexcylVertObj;
        osg::Shader*  ZtexcylFragObj;

        ZtexcylProgram = new osg::Program;
        ZtexcylProgram->setName( "ztexcyl" );
        ZtexcylVertObj = new osg::Shader( osg::Shader::VERTEX );
        ZtexcylFragObj = new osg::Shader( osg::Shader::FRAGMENT );
        ZtexcylProgram->addShader( ZtexcylFragObj );
        ZtexcylProgram->addShader( ZtexcylVertObj );

        load_shader_source( ZtexcylVertObj, "ztexcyl.vert" );
        load_shader_source( ZtexcylFragObj, "ztexcyl.frag" );

        cylStateSet->setAttributeAndModes(ZtexcylProgram, osg::StateAttribute::ON);
        tex_v_offset_uniform = new osg::Uniform( osg::Uniform::FLOAT, "tex_v_offset" );
        tex_v_offset_uniform->set(0.0f);
        cylStateSet->addUniform( tex_v_offset_uniform );

        osg::Uniform* tu = new osg::Uniform( osg::Uniform::SAMPLER_2D, "my_texture" );
        cylStateSet->addUniform( tu );
    }

    return myroot;
}

void StimulusCylinder::set_cylinder_rotation(float angle) {
    _angular_position_mode = true;
    _angular_position = angle;
}

void StimulusCylinder::set_cylinder_rotation_rate(float rate) {
    _angular_position_mode = false;
    _angular_velocity = rate;
}

void StimulusCylinder::set_cylinder_v_offset(float vo) {
    _v_offset_value_mode = true;
    _v_offset_value = vo;
}

void StimulusCylinder::set_cylinder_v_offset_rate(float vor) {
    _v_offset_value_mode = false;
    _v_offset_rate = vor;
}

void StimulusCylinder::set_cylinder_position(float x, float y, float z) {
    _cylinder->setCenter(osg::Vec3(x,y,z+(0.5*DEFAULT_HEIGHT)));
    dirty_cylinder();
}

void StimulusCylinder::set_cylinder_radius(float r) {
    _cylinder->setRadius(r);
    dirty_cylinder();
}

void StimulusCylinder::set_cylinder_height(float h) {
    _cylinder->setHeight(h);
    dirty_cylinder();
}

void StimulusCylinder::dirty_cylinder(void) {
    _shape->dirtyDisplayList();
    _shape->dirtyBound();
}

POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusCylinder)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
