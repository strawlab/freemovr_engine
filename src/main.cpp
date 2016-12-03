#include <string>

#include <osg/Vec3>
#include <osg/Quat>
#include <osg/ArgumentParser>
#include <osg/ApplicationUsage>

#include "Poco/Timestamp.h"
#include "Poco/Path.h"

#include "dsosg.h"

#include <iostream>

int main(int argc, char**argv) {
  dsosg::DSOSG *dsosg;
  Poco::Timestamp time;
  bool done;

  osg::ArgumentParser arguments(&argc, argv);
  arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
  arguments.getApplicationUsage()->setDescription("Manual display/camera calibration utility");
  arguments.getApplicationUsage()->addCommandLineOption("--config <filename>","Display server config JSON file");
  arguments.getApplicationUsage()->addCommandLineOption("--display-mode <name>","Display mode");

  osg::ApplicationUsage::Type help = arguments.readHelpType();
  if (help != osg::ApplicationUsage::NO_HELP) {
      arguments.getApplicationUsage()->write(std::cout);
      exit(0);
  }

  std::string config_filename = "~/freemoovr_engine-devel/freemoovr_engine/freemoovr_engine/config/config.json";
  while(arguments.read("--config", config_filename));

  std::string display_mode = "vr_display";
  while(arguments.read("--display-mode", display_mode));

  std::string freemoovr_basepath = "/home/john/Programming/freemoovr_engine.git/freemoovr_engine/";
  float observer_radius = 0.01;
  bool two_pass = false;
  bool show_geom_coords = false;

  osg::Vec3 observer_position(0,0,0);
  osg::Quat observer_orientation(0,0,0,1);

  dsosg = new dsosg::DSOSG(
                        freemoovr_basepath,
                        display_mode,
                        observer_radius,
                        config_filename,
                        two_pass,
                        show_geom_coords);

  dsosg->setup_viewer("display_server","{}");
  dsosg->set_stimulus_plugin("Stimulus3DDemo");

  done = false;
  while (!done) {
    float now;

    time.update();
    now = time.epochMicroseconds() * 1e6; /* microseconds to seconds */

    dsosg->update(now, observer_position, observer_orientation);
    dsosg->frame();
    done = dsosg->done();
  }

	return 0;
}
