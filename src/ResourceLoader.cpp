#include "freemoovr_engine/ResourceLoader.hpp"

#include <sstream>
#include <stdexcept>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include "Poco/Path.h"
#include "Poco/File.h"

#include "freemoovr_engine/freemoovr_assert.h"

ResourceLoader::ResourceLoader()
{
}

std::string ResourceLoader::get_plugin_shader_path(std::string name) const
{
    Poco::Path path(_freemoovr_base_path);
    path.makeDirectory();
    path.append("src").append("shaders").append(name);
    return path.absolute().toString();
}

std::string ResourceLoader::get_plugin_data_path(std::string name) const
{
    Poco::Path path(_freemoovr_base_path);
    path.makeDirectory();
    path.append("data").append(name);
    return path.absolute().toString();
}

osg::Node* ResourceLoader::load_osg_file(std::string name, bool throw_on_failure) const
{
    // FIXME: We should do this (and load_shader_source) using the osgDB resource
    //        framework. When set_xxx_path is called, add the appropriate subdirs
    //        (data and src/shaders) to the osgDB, and then query them here
    Poco::Path path(get_plugin_data_path(name));
    if (throw_on_failure) {
      if (!Poco::File(path).exists()) {
        std::ostringstream os;
        os << "Could not load OSG file, does not exist " << path.toString();
        throw std::runtime_error(os.str());
      }
    }
    osg::Node* result = osgDB::readNodeFile(path.absolute().toString());
    if (throw_on_failure) {
      if (result==NULL) {
        std::ostringstream os;
        os << "Could not load OSG file, exists but failed " << path.toString();
        throw std::runtime_error(os.str());
      }
    }
    return result;
}

osg::Image* ResourceLoader::load_image_file(std::string name, bool throw_on_failure) const
{
    // FIXME: We should do this (and load_shader_source) using the osgDB resource
    //        framework. When set_xxx_path is called, add the appropriate subdirs
    //        (data and src/shaders) to the osgDB, and then query them here
    Poco::Path path(get_plugin_data_path(name));
    if (throw_on_failure) {
      if (!Poco::File(path).exists()) {
        std::ostringstream os;
        os << "Could not load image file, does not exist " << path.toString();
        throw std::runtime_error(os.str());
      }
    }
    osg::Image *result = osgDB::readImageFile(path.absolute().toString());
    if (throw_on_failure) {
      if (result==NULL) {
        std::ostringstream os;
        os << "Could not load image file, exists but failed " << path.toString();
        throw std::runtime_error(os.str());
      }
    }
    return result;
}

void ResourceLoader::load_shader_source(osg::Shader* shader, std::string name) const
{
    Poco::Path path(get_plugin_shader_path(name));
    if (!Poco::File(path).exists()) {
        std::ostringstream os;
        os << "Could not load shader file, does not exist " << path.toString();
        throw std::runtime_error(os.str());
    }
    shader->loadShaderSourceFromFile(path.absolute().toString());
}

void ResourceLoader::set_freemoovr_base_path(std::string path) {
  _freemoovr_base_path = path;
}
