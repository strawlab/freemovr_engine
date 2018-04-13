/* -*- Mode: C++ -*- */
#ifndef ResourceLoader_INCLUDED
#define ResourceLoader_INCLUDED

#include <string>
#include <osg/Node>
#include <osg/Shader>

class ResourceLoader
{
 public:
  ResourceLoader(std::string package_share_dir);
  virtual ~ResourceLoader() {}

  virtual std::string get_plugin_shader_path(std::string name) const;
  virtual std::string get_plugin_data_path(std::string name) const;
  virtual osg::Node* load_osg_file(std::string name, bool throw_on_failure=true) const;
  virtual osg::Image* load_image_file(std::string name, bool throw_on_failure=true) const;
  virtual void load_shader_source(osg::Shader* shader, std::string name) const;

protected:

  std::string _package_share_dir;
};

#endif // ResourceLoader.hpp
