/* -*- Mode: C++ -*- */
#ifndef ResourceLoader_INCLUDED
#define ResourceLoader_INCLUDED

#include <string>
#include <osg/Node>
#include <osg/Shader>

class ResourceLoader
{
 public:
  ResourceLoader();
  virtual ~ResourceLoader() {}

  // This is called when initializing the plugin to tell it where to find flyvr's data.
  virtual void set_flyvr_base_path(std::string path);

  // This is called when initializing the plugin to tell it its path
  virtual void set_plugin_path(std::string path, bool popdir=true);

  virtual std::string get_plugin_shader_path(std::string name);
  virtual std::string get_plugin_data_path(std::string name);
  virtual osg::Node* load_osg_file(std::string name, bool throw_on_failure=true);
  virtual osg::Image* load_image_file(std::string name, bool throw_on_failure=true);
  virtual void load_shader_source(osg::Shader* shader, std::string name);

protected:

  std::string _flyvr_base_path;
  std::string _plugin_path;
  bool _popdir;
};

#endif // ResourceLoader.h
