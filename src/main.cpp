#include <string>

#include <osg/Vec3>
#include <osg/Quat>
#include <osg/ArgumentParser>
#include <osg/ApplicationUsage>

#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/TCPServer.h>
#include <Poco/Net/TCPServerConnection.h>
#include <Poco/Net/TCPServerConnectionFactory.h>
#include <Poco/Timestamp.h>
#include <Poco/Path.h>

#include <jansson.h>

#include "dsosg.h"
#include "json2osg.hpp"

#include <iostream>

class MyConnection : public Poco::Net::TCPServerConnection {
private:
    dsosg::DSOSG *_dsosg;
public:
    MyConnection(const Poco::Net::StreamSocket &socket, dsosg::DSOSG *dsosg)
        : Poco::Net::TCPServerConnection(socket), _dsosg(dsosg) {}

    virtual ~MyConnection() {}

    virtual void run()
    {
        std::cerr << "RUNNING\n";
        static const char message[] = "This is TCP server sample\n";
        socket().sendBytes(message, sizeof(message) - 1);
    }
};

class MyConnectionFactory : public Poco::Net::TCPServerConnectionFactory {
private:
    dsosg::DSOSG *_dsosg;
public:
    MyConnectionFactory(dsosg::DSOSG *dsosg) : _dsosg(dsosg) {}

    virtual ~MyConnectionFactory() {}

    virtual Poco::Net::TCPServerConnection* createConnection(const Poco::Net::StreamSocket &socket)
    {
        std::cerr << "CONNNECTED\n";
        return new MyConnection(socket, _dsosg);
    }
};

int main(int argc, char**argv) {
  dsosg::DSOSG *dsosg;
  Poco::Timestamp time;
  bool done;
  json_t *conf;
  json_error_t error;
  std::string display = "{}";

  static const Poco::UInt16 SERVER_PORT = 8888;
  Poco::Net::ServerSocket sock(SERVER_PORT);

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

  std::string config_filename = "config/config.json";
  while(arguments.read("--config", config_filename));

  std::string display_mode = "vr_display";
  while(arguments.read("--display-mode", display_mode));

  std::string flyvr_basepath = "";
  float observer_radius = 0.01;
  bool two_pass = false;
  bool show_geom_coords = false;

  osg::Vec3 observer_position(0,0,0);
  osg::Quat observer_orientation(0,0,0,1);

  dsosg = new dsosg::DSOSG(
                        flyvr_basepath,
                        display_mode,
                        observer_radius,
                        config_filename,
                        two_pass,
                        show_geom_coords);

  conf = json_load_file(config_filename.c_str(), 0, &error);
  if (conf) {
    json_t *json = json_object_get(conf, "display");
    if (json) {
        display = json_dumps(json, 0);
    }
  }

  dsosg->setup_viewer("display_server", display);
  dsosg->set_stimulus_plugin("Stimulus3DDemo");

  sock.listen();
  Poco::Net::TCPServer server(new MyConnectionFactory(dsosg), sock);
  server.start();

  done = false;
  while (!done) {
    float now;

    time.update();
    now = time.epochMicroseconds() * 1e6; /* microseconds to seconds */

    dsosg->update(now, observer_position, observer_orientation);
    dsosg->frame();
    done = dsosg->done();
  }

  server.stop();
  sock.close();

  return 0;
}
