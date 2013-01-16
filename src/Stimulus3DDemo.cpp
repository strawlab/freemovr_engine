/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#include "flyvr/stimulus_interface.h"

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

#include "util.h"

class Stimulus3DDemo: public StimulusInterface
{
public:
std::string name() const {
	return "Stimulus3DDemo";
}

virtual void post_init(void) {
	_virtual_world = create_virtual_world();
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
	return result;
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
}

std::string get_message_type(const std::string& topic_name) const {
	throw 0; // TODO: better error handling
}

osg::ref_ptr<osg::Group> create_virtual_world()
{

  osg::ref_ptr<osg::MatrixTransform> myroot = new osg::MatrixTransform; myroot->addDescription("virtual world root node");
  add_default_skybox(myroot);

  // Create a geometry transform node enabling use cut-and-pasted
  // geometry from OSG example and have it on the XY plane with Z
  // pointing up.
  osg::ref_ptr<osg::MatrixTransform> geom_transform_node = new osg::MatrixTransform;
  geom_transform_node->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(90.0),1.0,0.0,0.0));

  // Now populate this node with geometry.
  osg::ref_ptr<osg::Geode> geode_1 = new osg::Geode;
  geom_transform_node->addChild(geode_1.get());

  osg::ref_ptr<osg::Geode> geode_2 = new osg::Geode;
  osg::ref_ptr<osg::MatrixTransform> transform_2 = new osg::MatrixTransform;
  transform_2->addChild(geode_2.get());
  geom_transform_node->addChild(transform_2.get());

  const float radius = 0.8f;
  const float height = 1.0f;
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(2.0f);
  osg::ref_ptr<osg::ShapeDrawable> shape;

  shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-3.0f, 0.3f, 0.0f), radius), hints.get());
  shape->setColor(osg::Vec4(0.6f, 0.8f, 0.8f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new osg::ShapeDrawable(new osg::Cone(osg::Vec3(3.0f, 0.3f, 0.0f), 2 * radius, height), hints.get());
  shape->setColor(osg::Vec4(0.4f, 0.9f, 0.3f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0.0f, 0.3f, -3.0f), radius, height), hints.get());
  shape->setColor(osg::Vec4(0.2f, 0.5f, 0.7f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.3f, 3.0f), radius), hints.get());
  shape->setColor(osg::Vec4(0.5f, 0.3f, 0.5f, 1.0f));
  geode_2->addDrawable(shape.get());

  // material
  osg::ref_ptr<osg::Material> material = new osg::Material;
  material->setColorMode(osg::Material::DIFFUSE);
  material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
  material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
  material->setShininess(osg::Material::FRONT_AND_BACK, 64.0f);
  geom_transform_node->getOrCreateStateSet()->setAttributeAndModes(material.get(), osg::StateAttribute::ON);

  {
	  osg::ref_ptr<osg::Light> _light = new osg::Light;
	  _light->setLightNum(0);
	  _light->setAmbient(osg::Vec4(0.00f,0.0f,0.00f,1.0f));
	  _light->setDiffuse(osg::Vec4(0.8f,0.8f,0.8f,1.0f));
	  _light->setSpecular(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
	  _light->setPosition(osg::Vec4(2.0, 2.0, 5.0, 1.0));

	  geom_transform_node->getOrCreateStateSet()->setAssociatedModes(_light.get(),osg::StateAttribute::ON);

	  // enable lighting by default.
	  geom_transform_node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

  }
  myroot->addChild(geom_transform_node);
  return myroot;
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
