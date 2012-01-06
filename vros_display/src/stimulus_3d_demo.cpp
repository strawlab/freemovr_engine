/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#include "vros_display/stimulus_interface.h"

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

#include <boost/filesystem.hpp>

#include "util.h"

class Stimulus3DDemo: public StimulusInterface
{
public:
std::string name() const {
	return "Stimulus3DDemo";
}

virtual void post_init(std::string config_data_dir) {
	_virtual_world = create_virtual_world();//_vros_display_base_path);
	std::cout << "Stimulus3DDemo: _virtual_world created" << std::endl;
}

void resized(int width,int height) {
}

osg::ref_ptr<osg::Group> get_3d_world() {
    return _virtual_world;
}

osg::ref_ptr<osg::Group> get_2d_hud() {
    return 0;
}

std::vector<std::string> get_topic_names() {
	std::vector<std::string> result;
	return result;
}

void send_json_message(std::string topic_name, std::string json_message) {
}

std::string get_message_type(std::string topic_name) {
	throw 0; // TODO: better error handling
}

osg::ref_ptr<osg::Group> create_virtual_world()
{

  namespace fs = boost::filesystem;
  osg::ref_ptr<osg::MatrixTransform> top = new osg::MatrixTransform; top->addDescription("virtual world top node");

  add_default_skybox(top);
  top->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(90.0),1.0,0.0,0.0));

  osg::ref_ptr<osg::Geode> geode_1 = new osg::Geode;
  top->addChild(geode_1.get());

  osg::ref_ptr<osg::Geode> geode_2 = new osg::Geode;
  osg::ref_ptr<osg::MatrixTransform> transform_2 = new osg::MatrixTransform;
  transform_2->addChild(geode_2.get());
  top->addChild(transform_2.get());

  const float radius = 0.8f;
  const float height = 1.0f;
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(2.0f);
  osg::ref_ptr<osg::ShapeDrawable> shape;

  shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, -10.0f, 0.0f), 8.0f), hints.get());
  shape->setColor(osg::Vec4(0.5f, 0.5f, 0.7f, 1.0f));
  geode_1->addDrawable(shape.get());

  shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-3.0f, 0.0f, 0.0f), radius), hints.get());
  shape->setColor(osg::Vec4(0.6f, 0.8f, 0.8f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new osg::ShapeDrawable(new osg::Cone(osg::Vec3(3.0f, 0.0f, 0.0f), 2 * radius, height), hints.get());
  shape->setColor(osg::Vec4(0.4f, 0.9f, 0.3f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0.0f, 0.0f, -3.0f), radius, height), hints.get());
  shape->setColor(osg::Vec4(0.2f, 0.5f, 0.7f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 3.0f), radius), hints.get());
  shape->setColor(osg::Vec4(0.5f, 0.3f, 0.5f, 1.0f));
  geode_2->addDrawable(shape.get());

  // material
  osg::ref_ptr<osg::Material> material = new osg::Material;
  material->setColorMode(osg::Material::DIFFUSE);
  material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
  material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
  material->setShininess(osg::Material::FRONT_AND_BACK, 64.0f);
  top->getOrCreateStateSet()->setAttributeAndModes(material.get(), osg::StateAttribute::ON);

  {
	  osg::ref_ptr<osg::Light> _light = new osg::Light;
	  _light->setLightNum(0);
	  _light->setAmbient(osg::Vec4(0.00f,0.0f,0.00f,1.0f));
	  _light->setDiffuse(osg::Vec4(0.8f,0.8f,0.8f,1.0f));
	  _light->setSpecular(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
	  _light->setPosition(osg::Vec4(2.0, 2.0, 5.0, 1.0));

	  top->getOrCreateStateSet()->setAssociatedModes(_light.get(),osg::StateAttribute::ON);

	  osg::LightModel* lightmodel = new osg::LightModel;
	  lightmodel->setAmbientIntensity(osg::Vec4(0.1f,0.1f,0.1f,1.0f));
	  top->getOrCreateStateSet()->setAttributeAndModes(lightmodel, osg::StateAttribute::ON);

	  // enable lighting by default.
	  top->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

  }
  return top;
}

private:
	osg::ref_ptr<osg::Group> _virtual_world;
};


POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(Stimulus3DDemo)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
