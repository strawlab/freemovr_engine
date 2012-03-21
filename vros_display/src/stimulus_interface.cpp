// This is used both by the class library and by the application.
#include "vros_display/stimulus_interface.h"
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

#include <stdexcept>

std::string mkfname(std::string basepath, std::string middle, std::string extension) {
  Poco::Path path(basepath);
  path.makeDirectory();
  path.append(Poco::Path(middle));

  if (extension.at(0) == '.') {
    extension = extension.substr(1,extension.length());
  }
  path.setExtension(extension);

  return path.toString();
}

StimulusInterface::StimulusInterface() :  _skybox_pat(NULL)
{
}

StimulusInterface::~StimulusInterface()
{
}

void StimulusInterface::set_vros_display_base_path(std::string bp) {
  _vros_display_base_path = bp;
}

void StimulusInterface::update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation ) {
  if (_skybox_pat.valid()) {
    // this is a skybox - don't update the orientation with the observer
    _skybox_pat->setPosition(observer_position);
  }
}

void StimulusInterface::add_default_skybox(osg::ref_ptr<osg::Group> top) {
  Poco::Path base_path(_vros_display_base_path);

  base_path.makeDirectory();
  base_path.pushDirectory("sample_data");
  base_path.pushDirectory("Pond");

  std::string basepath = base_path.toString();
  std::string extension = "jpg";

  add_skybox(top, basepath, extension);
}

osg::Vec4 StimulusInterface::get_clear_color() const {
  return osg::Vec4(0.0, 0.0, 0.0, 0.0); // default to transparent black
}

void StimulusInterface::add_skybox(osg::ref_ptr<osg::Group> top, std::string basepath, std::string extension) {
  if (1) {
	  // add skybox
    std::string example = mkfname(basepath,"posx",extension);
    if (!Poco::File(Poco::Path(example)).exists()) {
      std::cerr << "skymap file like " << example << "do not exist" << std::endl;
    }

	  osg::Image* posx = osgDB::readImageFile(mkfname(basepath,"posx",extension));
	  osg::Image* posy = osgDB::readImageFile(mkfname(basepath,"posy",extension));
	  osg::Image* posz = osgDB::readImageFile(mkfname(basepath,"posz",extension));
	  osg::Image* negx = osgDB::readImageFile(mkfname(basepath,"negx",extension));
	  osg::Image* negy = osgDB::readImageFile(mkfname(basepath,"negy",extension));
	  osg::Image* negz = osgDB::readImageFile(mkfname(basepath,"negz",extension));

	  if (!posx) throw std::runtime_error("could not read skymap file");
	  if (!posy) throw std::runtime_error("could not read skymap file");
	  if (!posz) throw std::runtime_error("could not read skymap file");
	  if (!negx) throw std::runtime_error("could not read skymap file");
	  if (!negy) throw std::runtime_error("could not read skymap file");
	  if (!negz) throw std::runtime_error("could not read skymap file");

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

		  std::string shader_dir = join_path( _vros_display_base_path, "src/shaders" );
		  LoadShaderSource( ShowCubemapVertObj, join_path(shader_dir,"skybox.vert" ));
		  LoadShaderSource( ShowCubemapFragObj, join_path(shader_dir, "skybox.frag" ));

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
