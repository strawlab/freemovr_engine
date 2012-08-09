#ifndef load_cubemap_INCLUDES
#define load_cubemap_INCLUDES
#include <osg/TextureCubeMap>
#include <string>

osg::TextureCubeMap* load_cubemap(std::string basepath, std::string extension);
#endif
