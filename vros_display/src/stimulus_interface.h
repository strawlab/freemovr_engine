// This is used both by the class library and by the application.
#ifndef StimulusInterface_INCLUDED
#define StimulusInterface_INCLUDED

#include <iostream>
#include <osg/Group>

class StimulusInterface
{
 public:
  StimulusInterface();
  virtual ~StimulusInterface();
  virtual void post_init(std::string config_data_dir, std::string shader_dir) = 0;
  virtual void resized(int width,int height) = 0;
  virtual osg::ref_ptr<osg::Group> get_3d_world() = 0;
  virtual osg::ref_ptr<osg::Group> get_2d_hud() = 0;
  virtual std::string name() const = 0;

  virtual std::vector<std::string> get_topic_names() = 0;
  virtual std::string get_message_type(std::string topic_name) = 0;
  virtual void send_json_message(std::string topic_name, std::string json_message) = 0;
};
#endif // StimulusInterface.h
