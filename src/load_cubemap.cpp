#include "load_cubemap.h"

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

osg::TextureCubeMap* load_cubemap(std::string basepath, std::string extension) {
    std::string example = mkfname(basepath,"posx",extension);
    if (!Poco::File(Poco::Path(example)).exists()) {
      std::cerr << "skymap file like " << example << " does not exist" << std::endl;
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
                  return skymap;
}
