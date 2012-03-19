// This is used both by the class library and by the application.
#ifndef StimulusInterface_INCLUDED
#define StimulusInterface_INCLUDED

#include <iostream>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>

class StimulusInterface
{
 public:
  StimulusInterface();
  virtual ~StimulusInterface();

  // This is called when initializing the plugin to tell it where to find vros_display's data.
  virtual void set_vros_display_base_path(std::string vros_display_base_path);

  virtual void post_init(std::string config_data_dir) {};
  virtual void resized(int width,int height) {};
  virtual osg::ref_ptr<osg::Group> get_3d_world() {return 0;}
  virtual osg::ref_ptr<osg::Group> get_2d_hud() {return 0;}
  virtual std::string name() const = 0;
  virtual osg::Vec4 get_clear_color() const;

  virtual void update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation );

  // what topics does the plugin subscribe to?
  virtual std::vector<std::string> get_topic_names() const = 0;

  // what is their type? (e.g. topic "my_stamp" might result in "std_msgs/Header")
  virtual std::string get_message_type(const std::string& topic_name) const = 0;

  // and here is the message data, as a JSON message.
  virtual void receive_json_message(const std::string& topic_name, const std::string& json_message) = 0;

 protected:
  // your derived class can call this if you want to add a skybox
  virtual void add_skybox(osg::ref_ptr<osg::Group> top, std::string image_basepath, std::string image_extension);
  virtual void add_default_skybox(osg::ref_ptr<osg::Group> top);

  osg::ref_ptr<osg::PositionAttitudeTransform> _skybox_pat;

 private:
  std::string _vros_display_base_path;
};

#endif // StimulusInterface.h
