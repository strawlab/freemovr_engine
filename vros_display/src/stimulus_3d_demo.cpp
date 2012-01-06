/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#include "stimulus_interface.h"

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

void add_skybox(osg::ref_ptr<osg::Group> top, std::string config_data_dir, std::string shader_dir) {
 if (1) {
	  // add skybox
	  osg::Image* posx = osgDB::readImageFile(join_path(config_data_dir, "Pond/posx.jpg"));
	  osg::Image* posy = osgDB::readImageFile(join_path(config_data_dir, "Pond/posy.jpg"));
	  osg::Image* posz = osgDB::readImageFile(join_path(config_data_dir, "Pond/posz.jpg"));
	  osg::Image* negx = osgDB::readImageFile(join_path(config_data_dir, "Pond/negx.jpg"));
	  osg::Image* negy = osgDB::readImageFile(join_path(config_data_dir, "Pond/negy.jpg"));
	  osg::Image* negz = osgDB::readImageFile(join_path(config_data_dir, "Pond/negz.jpg"));

	  posx->flipVertical();
	  posy->flipVertical();
	  posz->flipVertical();
	  negx->flipVertical();
	  negy->flipVertical();
	  negz->flipVertical();

	  if (1) {
		  osg::TextureCubeMap* skymap = new osg::TextureCubeMap;
		  skymap->setImage(osg::TextureCubeMap::POSITIVE_X, posx);
		  skymap->setImage(osg::TextureCubeMap::NEGATIVE_X, negx);
		  skymap->setImage(osg::TextureCubeMap::POSITIVE_Y, posy);
		  skymap->setImage(osg::TextureCubeMap::NEGATIVE_Y, negy);
		  skymap->setImage(osg::TextureCubeMap::POSITIVE_Z, posz);
		  skymap->setImage(osg::TextureCubeMap::NEGATIVE_Z, negz);
		  skymap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
		  skymap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
		  skymap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);

		  skymap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
		  skymap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);

		  osg::Group* group = new osg::Group;
		  osg::Geode* geode = new osg::Geode();
		  osg::ref_ptr<osg::ShapeDrawable> box = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 100.0));
		  geode->addDrawable(box);
		  group->addChild(geode);

		  std::vector< osg::ref_ptr<osg::Program> > _programList;
		  osg::Program* ShowCubemapProgram;
		  osg::Shader*  ShowCubemapVertObj;
		  osg::Shader*  ShowCubemapFragObj;

		  ShowCubemapProgram = new osg::Program;
		  ShowCubemapProgram->setName( "skybox" );
		  _programList.push_back( ShowCubemapProgram );
		  ShowCubemapVertObj = new osg::Shader( osg::Shader::VERTEX );
		  ShowCubemapFragObj = new osg::Shader( osg::Shader::FRAGMENT );
		  ShowCubemapProgram->addShader( ShowCubemapFragObj );
		  ShowCubemapProgram->addShader( ShowCubemapVertObj );

		  LoadShaderSource( ShowCubemapVertObj, join_path(shader_dir,"skybox.vert" ));
		  LoadShaderSource( ShowCubemapFragObj, join_path(shader_dir, "skybox.frag" ));

		  osg::Uniform* skymapSampler = new osg::Uniform( osg::Uniform::SAMPLER_CUBE, "skybox" );

		  osg::StateSet* ss = geode->getOrCreateStateSet();
		  ss->setAttributeAndModes(ShowCubemapProgram, osg::StateAttribute::ON);
		  ss->addUniform( skymapSampler );
		  ss->setTextureAttributeAndModes(0,skymap,osg::StateAttribute::ON);
		  ss->setMode(GL_BLEND, osg::StateAttribute::ON);
		  ss->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::ON);

		  top->addChild(group);
	  }

  }
 }


namespace osg
{

osg::ref_ptr<osg::Group> create_virtual_world(std::string config_data_dir, std::string shader_dir)
{

  ref_ptr<MatrixTransform> top = new MatrixTransform; top->addDescription("virtual world top node");
  add_skybox(top,config_data_dir,shader_dir);
  top->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(90.0),1.0,0.0,0.0));

  ref_ptr<Geode> geode_1 = new Geode;
  top->addChild(geode_1.get());

  ref_ptr<Geode> geode_2 = new Geode;
  ref_ptr<MatrixTransform> transform_2 = new MatrixTransform;
  transform_2->addChild(geode_2.get());
  //transform_2->setUpdateCallback(new osg::AnimationPathCallback(Vec3(0, 0, 0), Y_AXIS, inDegrees(45.0f)));
  top->addChild(transform_2.get());

  //  ref_ptr<Geode> geode_3 = new Geode;
  //  ref_ptr<MatrixTransform> transform_3 = new MatrixTransform;
  //  transform_3->addChild(geode_3.get());
  //  transform_3->setUpdateCallback(new osg::AnimationPathCallback(Vec3(0, 0, 0), Y_AXIS, inDegrees(-22.5f)));
  //  top->addChild(transform_3.get());

  const float radius = 0.8f;
  const float height = 1.0f;
  ref_ptr<TessellationHints> hints = new TessellationHints;
  hints->setDetailRatio(2.0f);
  ref_ptr<ShapeDrawable> shape;

  //shape = new ShapeDrawable(new Box(Vec3(0.0f, -2.0f, 0.0f), 10, 0.1f, 10), hints.get());
  shape = new ShapeDrawable(new Sphere(Vec3(0.0f, -10.0f, 0.0f), 8.0f), hints.get());
  shape->setColor(Vec4(0.5f, 0.5f, 0.7f, 1.0f));
  geode_1->addDrawable(shape.get());


  shape = new ShapeDrawable(new Sphere(Vec3(-3.0f, 0.0f, 0.0f), radius), hints.get());
  shape->setColor(Vec4(0.6f, 0.8f, 0.8f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new ShapeDrawable(new Cone(Vec3(3.0f, 0.0f, 0.0f), 2 * radius, height), hints.get());
  shape->setColor(Vec4(0.4f, 0.9f, 0.3f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new ShapeDrawable(new Cone(Vec3(0.0f, 0.0f, -3.0f), radius, height), hints.get());
  shape->setColor(Vec4(0.2f, 0.5f, 0.7f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new ShapeDrawable(new Sphere(Vec3(0.0f, 0.0f, 3.0f), radius), hints.get());
  shape->setColor(Vec4(0.5f, 0.3f, 0.5f, 1.0f));
  geode_2->addDrawable(shape.get());

  //  shape = new ShapeDrawable(new Box(Vec3(0.0f, 3.0f, 0.0f), 2, 0.1f, 2), hints.get());
  //  shape->setColor(Vec4(0.8f, 0.8f, 0.4f, 1.0f));
  //  geode_3->addDrawable(shape.get());

  // material
  ref_ptr<Material> material = new Material;
  material->setColorMode(Material::DIFFUSE);
  material->setAmbient(Material::FRONT_AND_BACK, Vec4(0, 0, 0, 1));
  material->setSpecular(Material::FRONT_AND_BACK, Vec4(1, 1, 1, 1));
  material->setShininess(Material::FRONT_AND_BACK, 64.0f);
  top->getOrCreateStateSet()->setAttributeAndModes(material.get(), StateAttribute::ON);

  {
	  osg::ref_ptr<osg::Light> _light = new osg::Light;
	  _light->setLightNum(0);
	  _light->setAmbient(Vec4(0.00f,0.0f,0.00f,1.0f));
	  _light->setDiffuse(Vec4(0.8f,0.8f,0.8f,1.0f));
	  _light->setSpecular(Vec4(1.0f,1.0f,1.0f,1.0f));
	  _light->setPosition(osg::Vec4d(2.0, 2.0, 5.0, 1.0));

	  top->getOrCreateStateSet()->setAssociatedModes(_light.get(),osg::StateAttribute::ON);

	  osg::LightModel* lightmodel = new osg::LightModel;
	  lightmodel->setAmbientIntensity(osg::Vec4(0.1f,0.1f,0.1f,1.0f));
	  top->getOrCreateStateSet()->setAttributeAndModes(lightmodel, osg::StateAttribute::ON);

	  // enable lighting by default.
	  top->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

  }
  return top;
}
};


class Stimulus3DDemo: public StimulusInterface
{
public:
std::string name() const {
	return "Stimulus3DDemo";
}

virtual void post_init(std::string config_data_dir, std::string shader_dir) {
	_virtual_world = osg::create_virtual_world(config_data_dir,shader_dir);
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

private:
	osg::ref_ptr<osg::Group> _virtual_world;
};


POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(Stimulus3DDemo)
POCO_END_MANIFEST

// optional set up and clean up functions
void pocoInitializeLibrary()
{
  std::cout << "Stimulus3DDemo initializing" << std::endl;
}

void pocoUninitializeLibrary()
{
  std::cout << "Stimulus3DDemo uninitializing" << std::endl;
}
