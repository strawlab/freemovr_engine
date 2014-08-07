/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

/* StimulusSphere
   Minimal example of how to draw flat (no lighting effects) objects
   in a 3D world.
 */

#include "flyvr/StimulusInterface.hpp"
#include "Poco/ClassLibrary.h"

#include <stdexcept>
#include <iostream>
#include <vector>
#include <string>

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

class StimulusSphere: public StimulusInterface
{
public:
    StimulusSphere() {};

    void post_init(bool);
    std::vector<std::string> get_topic_names() const { return std::vector<std::string>(); }
    std::string get_message_type(const std::string& topic_name) const { throw std::runtime_error("no message types to get"); }
    std::string name() const { return "StimulusCylinder"; }
    void receive_json_message(const std::string&, const std::string&) {}
    osg::ref_ptr<osg::Group> get_3d_world() { return _virtual_world; }
    osg::Vec4 get_clear_color() const { return osg::Vec4(0.0, 0.0, 0.0, 1.0); } //black background

private:
osg::ref_ptr<osg::Group> _virtual_world;
};

void StimulusSphere::post_init(bool) {

  osg::ref_ptr<osg::MatrixTransform> root = new osg::MatrixTransform;
  root->addDescription("root node");
  //disable all lighting in the scene so objects always appear the same color.
  root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

  osg::ref_ptr<osg::MatrixTransform> shape_transform = new osg::MatrixTransform;
  shape_transform->setName("sphere");
  osg::ref_ptr<osg::Geode> shape_geode = new osg::Geode;

  osg::ref_ptr<osg::ShapeDrawable> shape;
  shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.1f, 0.0f, 0.0f), 0.1));
  shape->setColor(osg::Vec4(0.6f, 0.8f, 0.8f, 1.0f));

  shape_geode->addDrawable(shape.get());
  shape_transform->addChild(shape_geode.get());
  root->addChild(shape_transform.get());

  _virtual_world = root;
}

POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusSphere)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
