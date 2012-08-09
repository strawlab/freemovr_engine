#include "vros_display/ResourceLoader.h"

#include <sstream>
#include <stdexcept>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include "Poco/Path.h"
#include "Poco/File.h"

#include "vros_display/vros_assert.h"

ResourceLoader::ResourceLoader()
{
}

std::string ResourceLoader::get_plugin_shader_path(std::string name)
{
    Poco::Path path(_plugin_path);
    path.makeDirectory();
    if (_popdir) {
      vros_assert_msg( path.depth() > 0,
                       "_plugin_path does not include directory. (Is it set?)" );
      path.popDirectory();
    }
    path.append("src").append("shaders").append(name);
    return path.absolute().toString();
}

std::string ResourceLoader::get_plugin_data_path(std::string name)
{
    Poco::Path path(_plugin_path);
    path.makeDirectory();
    if (_popdir) {
      vros_assert_msg( path.depth() > 0,
                       "_plugin_path does not include directory. (Is it set?)" );
      path.popDirectory();
    }
    path.append("data").append(name);
    return path.absolute().toString();
}

osg::Node* ResourceLoader::load_osg_file(std::string name, bool throw_on_failure)
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
void ResourceLoader::load_shader_source(osg::Shader* shader, std::string name)
{
    Poco::Path path(get_plugin_shader_path(name));
    if (!Poco::File(path).exists()) {
        std::ostringstream os;
        os << "Could not load shader file, does not exist " << path.toString();
        throw std::runtime_error(os.str());
    }
    shader->loadShaderSourceFromFile(path.absolute().toString());
}

void ResourceLoader::set_vros_display_base_path(std::string path) {
  _vros_display_base_path = path;
}

void ResourceLoader::set_plugin_path(std::string path,bool popdir) {
  _plugin_path = path;
  _popdir = popdir;
}
