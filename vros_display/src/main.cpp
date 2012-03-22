#include <string>

#include <osg/Vec3>
#include <osg/Quat>

#include "Poco/Timestamp.h"

#include "dsosg.h"

int main(int argc, char**argv) {
  dsosg::DSOSG *dsosg;
  Poco::Timestamp time;
  bool done;

  std::string vros_display_basepath = "/home/stowers/vros-devel/vros/vros_display";
  std::string config_file = "/home/stowers/vros-devel/vros/vros_display/sample_data/config.json";
  std::string mode = "vr_display";
  float observer_radius = 0.01;
  bool two_pass = false;
  bool show_geom_coords = false;

  osg::Vec3 observer_position(0,0,0);
  osg::Quat observer_orientation(0,0,0,1);

  dsosg = new dsosg::DSOSG(
                        vros_display_basepath,
                        mode,
                        observer_radius,
                        config_file,
                        two_pass,
                        show_geom_coords);

  dsosg->setup_viewer("{}");
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
