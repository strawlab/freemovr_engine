// This is used both by the class library and by the application.
#include "freemoovr_engine/StimulusInterface.hpp"
#include "util.h"

#include <osg/MatrixTransform>
#include <osg/TextureCubeMap>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LightModel>
#include <osg/Depth>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include "Poco/Path.h"
#include "Poco/File.h"

#include "load_cubemap.h"
#include "InvalidBoundsCallback.h"

#include <stdexcept>

StimulusInterface::StimulusInterface() :  _skybox_pat(NULL)
{
}

StimulusInterface::~StimulusInterface()
{
}

bool StimulusInterface::is_CUDA_available() {
#ifdef FREEMOOVR_USE_CUDA
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
  Poco::Path base_path(_freemoovr_base_path);

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

      // Set up geometry for the background quad
      osg::Geometry* BackgroundGeometry = new osg::Geometry();

      // Force bounding box to be undefined, since we change vertex
      // position in the shader.
      osg::Drawable::ComputeBoundingBoxCallback* no_bounds_callback =
        new InvalidBoundsCallback();
      BackgroundGeometry->setComputeBoundingBoxCallback(no_bounds_callback);

      // vertices are interpreted as containing screen coordinates by
      // the vertex shader
      osg::Vec4Array* BackgroundVertices = new osg::Vec4Array(4);
      (*BackgroundVertices)[0].set(-1.0f,-1.0f, 1.0f, 1.0f);
      (*BackgroundVertices)[1].set( 1.0f,-1.0f, 1.0f, 1.0f);
      (*BackgroundVertices)[2].set( 1.0f, 1.0f, 1.0f, 1.0f);
      (*BackgroundVertices)[3].set(-1.0f, 1.0f, 1.0f, 1.0f);

      osg::DrawElementsUInt* BackgroundIndices =
        new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
      BackgroundIndices->push_back(0);
      BackgroundIndices->push_back(1);
      BackgroundIndices->push_back(2);
      BackgroundIndices->push_back(3);

      BackgroundGeometry->addPrimitiveSet(BackgroundIndices);
      BackgroundGeometry->setVertexArray(BackgroundVertices);

      geode->addDrawable(BackgroundGeometry);

      _skybox_pat->addChild(geode);

      // add vertex & fragment shaders, which make the quad fullscreen
      // and project the cubemap according to the view direction on it
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

          Poco::Path shader_path = Poco::Path(_freemoovr_base_path).append("src").append("shaders");
          ShowCubemapVertObj->loadShaderSourceFromFile(Poco::Path(shader_path).append("CubeBackground.vert").toString());
          ShowCubemapFragObj->loadShaderSourceFromFile(Poco::Path(shader_path).append("CubeBackground.frag").toString());

      osg::Uniform* skymapSampler = new osg::Uniform( osg::Uniform::SAMPLER_CUBE, "skybox" );

      osg::StateSet* ss = geode->getOrCreateStateSet();
      ss->setAttributeAndModes(ShowCubemapProgram, osg::StateAttribute::ON);
      ss->addUniform( skymapSampler );
      ss->setTextureAttributeAndModes(0,skymap,osg::StateAttribute::ON);

      // set depth test to LESS EQUAL because background shader renders at precisely zFar
      osg::Depth* depth = new osg::Depth;
      depth->setFunction(osg::Depth::LEQUAL);
      ss->setAttributeAndModes( depth, osg::StateAttribute::ON );

      top->addChild(_skybox_pat);
    }

  }
 }
