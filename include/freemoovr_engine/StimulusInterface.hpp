/* -*- Mode: C++ -*- */
// This is used both by the class library and by the application.
#ifndef StimulusInterface_INCLUDED
#define StimulusInterface_INCLUDED

#include "ResourceLoader.hpp"
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <freemoovr_engine/CallbackHolder.hpp>

class StimulusInterface: public ResourceLoader
{
 public:
  StimulusInterface();
  virtual ~StimulusInterface();

  // Initialization. Called after the plugin path has been set. Plugins can call get_plugin_shader_dir
  // and friends in here.
  virtual void post_init(bool slave=false) {};

  bool is_CUDA_available();

  // An event fired when the display window is resized.
  virtual void resized(int width,int height) {};

  // The plugin returns 3D part of scenegraph from this call.
  virtual osg::ref_ptr<osg::Group> get_3d_world() {return 0;}

  // The plugin returns the 2D overlay HUD (heads up display) from this call.
  virtual osg::ref_ptr<osg::Group> get_2d_hud() {return 0;}

  // The plugin returns its name in this call. Should match the class name.
  virtual std::string name() const = 0;

  // The plugin returns the color with which to clear the screen here.
  virtual osg::Vec4 get_clear_color() const;

  virtual void update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation );

  // what topics does the plugin subscribe to?
  virtual std::vector<std::string> get_topic_names() const = 0;

  // what is their type? (e.g. topic "my_stamp" might result in "std_msgs/Header")
  virtual std::string get_message_type(const std::string& topic_name) const = 0;

  // and here is the message data, as a JSON message.
  virtual void receive_json_message(const std::string& topic_name, const std::string& json_message) = 0;

  // If you need to change the background color, remember this object
  // and call its setBackgroundColorImplementation() method when the
  // background changes.
  virtual void set_background_color_callback(freemoovr_engine::BackgroundColorCallback*) {}

 protected:
  // your derived class can call this if you want to add a skybox
  virtual void add_skybox(osg::ref_ptr<osg::Group> top, std::string image_basepath, std::string image_extension);
  virtual void add_default_skybox(osg::ref_ptr<osg::Group> top);

  osg::ref_ptr<osg::PositionAttitudeTransform> _skybox_pat;
};

#endif // StimulusInterface.hpp
