/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "freemovr_engine/StimulusInterface.hpp"

#include "Poco/ClassLibrary.h"

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

class StimulusAxes: public StimulusInterface
{
public:
StimulusAxes(std::string p) : StimulusInterface(p) {}

std::string name() const {
	return "StimulusAxes";
}

virtual void post_init(bool slave) {
	_virtual_world = create_virtual_world();
}

void resized(int width,int height) {
}

void update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation ) {
    _virtual_world->setPosition(observer_position);
}

osg::ref_ptr<osg::Group> get_3d_world() {
    return _virtual_world;
}

osg::ref_ptr<osg::Group> get_2d_hud() {
    return 0;
}

std::vector<std::string> get_topic_names() const {
	std::vector<std::string> result;
	return result;
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
}

std::string get_message_type(const std::string& topic_name) const {
	throw 0; // TODO: better error handling
}

osg::ref_ptr<osg::PositionAttitudeTransform> create_virtual_world()
{
  osg::ref_ptr<osg::PositionAttitudeTransform> myroot = new osg::PositionAttitudeTransform;
  myroot->addDescription("virtual world root node");
  //disable all lighting in the scene so objects always appear the same color.
  myroot->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  myroot->addChild(geode.get());

  osg::ref_ptr<osg::ShapeDrawable> shape;

  float L = 0.5f;
  float l = 0.005f;
  osg::Vec3 center = osg::Vec3(0.0f, 0.0f, 0.0f);

  _plane_x = new osg::Box(center,l,L,L);
  shape = new osg::ShapeDrawable(_plane_x);
  shape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
  geode->addDrawable(shape.get());

  _plane_y = new osg::Box(center,L,l,L);
  shape = new osg::ShapeDrawable(_plane_y);
  shape->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
  geode->addDrawable(shape.get());

  _plane_z = new osg::Box(center,L,L,l);
  shape = new osg::ShapeDrawable(_plane_z);
  shape->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
  geode->addDrawable(shape.get());

  return myroot;
}

private:
    osg::ref_ptr<osg::Geode> _geode;
    osg::ref_ptr<osg::PositionAttitudeTransform>            _virtual_world;
    osg::ref_ptr<osg::Box> _plane_x;
    osg::ref_ptr<osg::Box> _plane_y;
    osg::ref_ptr<osg::Box> _plane_z;
};

MAKE_STIMULUS_INTERFACE_LOADER(StimulusAxes);

POCO_BEGIN_MANIFEST(StimulusInterfaceLoader)
POCO_EXPORT_CLASS(StimulusAxesLoader)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
