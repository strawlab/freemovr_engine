// This is used both by the class library and by the application.
#include "flyvr/StimulusInterface.hpp"
#include "util.h"

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

#include "Poco/Path.h"
#include "Poco/File.h"

#include "load_cubemap.h"

#include <stdexcept>

StimulusInterface::StimulusInterface() :  _skybox_pat(NULL)
{
}

StimulusInterface::~StimulusInterface()
{
}

bool StimulusInterface::is_CUDA_available() {
#ifdef FLYVR_USE_CUDA
  return true;
#else
  return false;
#endif
}

void StimulusInterface::update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation ) {
  if (_skybox_pat.valid()) {
    // this is a skybox - don't update the orientation with the observer
    _skybox_pat->setPosition(observer_position);
  }
}

void StimulusInterface::add_default_skybox(osg::ref_ptr<osg::Group> top) {
  Poco::Path base_path(_flyvr_base_path);

  base_path.makeDirectory();
  base_path.pushDirectory("data");
  base_path.pushDirectory("Pond");

  std::string basepath = base_path.toString();
  std::string extension = "jpg";

  add_skybox(top, basepath, extension);
}

osg::Vec4 StimulusInterface::get_clear_color() const {
  return osg::Vec4(1.0, 1.0, 0.0, 0.0); // default to clear yellow
}

void StimulusInterface::add_skybox(osg::ref_ptr<osg::Group> top, std::string basepath, std::string extension) {
  if (1) {
	  // add skybox

	  if (1) {
                  osg::TextureCubeMap* skymap = load_cubemap(basepath,extension);

		  _skybox_pat = new osg::PositionAttitudeTransform;
		  osg::Geode* geode = new osg::Geode();
		  osg::ref_ptr<osg::ShapeDrawable> box = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 100.0));
		  geode->addDrawable(box);
		  _skybox_pat->addChild(geode);

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

          Poco::Path shader_path = Poco::Path(_flyvr_base_path).append("src").append("shaders");
          ShowCubemapVertObj->loadShaderSourceFromFile(Poco::Path(shader_path).append("skybox.vert").toString());
          ShowCubemapFragObj->loadShaderSourceFromFile(Poco::Path(shader_path).append("skybox.frag").toString());

		  osg::Uniform* skymapSampler = new osg::Uniform( osg::Uniform::SAMPLER_CUBE, "skybox" );

		  osg::StateSet* ss = geode->getOrCreateStateSet();
		  ss->setAttributeAndModes(ShowCubemapProgram, osg::StateAttribute::ON);
		  ss->addUniform( skymapSampler );
		  ss->setTextureAttributeAndModes(0,skymap,osg::StateAttribute::ON);
		  ss->setMode(GL_BLEND, osg::StateAttribute::ON);
		  ss->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::ON);

		  top->addChild(_skybox_pat);
	  }

  }
 }
