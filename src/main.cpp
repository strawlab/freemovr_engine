#include <string.h>

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
#include <Poco/Mutex.h>

#include <jansson.h>

#include "dsosg.h"
#include "json2osg.hpp"

#include <iostream>
#include <queue>

struct StimulusMessage
{
    std::string topic;
    std::string json;
    StimulusMessage(const char *t, const char *j) : topic(t), json(j) {}
};

typedef std::queue<StimulusMessage> StimulusQueue;

class MyConnection : public Poco::Net::TCPServerConnection {
private:
    dsosg::DSOSG    *_dsosg;
    osg::Vec3       &_observer_position;
    osg::Quat       &_observer_orientation;
    Poco::Mutex     &_mutex;
    StimulusQueue   &_msgs;

public:
    MyConnection(const Poco::Net::StreamSocket &socket, dsosg::DSOSG *dsosg, osg::Vec3 &op, osg::Quat &oo, Poco::Mutex &m, StimulusQueue &msgs)
        : Poco::Net::TCPServerConnection(socket),
          _dsosg(dsosg),
          _observer_position(op),
          _observer_orientation(oo),
          _mutex(m),
          _msgs(msgs) {}

    virtual ~MyConnection() {}

    virtual void run()
    {
        char buf[4096];
        int ret = socket().receiveBytes(buf, sizeof(buf)-1);
        if (ret > 0) {
            json_t *json;
            json_error_t error;
            Poco::Mutex::ScopedLock lock(_mutex);

            buf[ret] = '\0';
            json = json_loads(buf, 0, &error);
            if (json) {
                const char *key;
                json_t *value;
                void *iter = json_object_iter(json);
                while(iter) {
                    key = json_object_iter_key(iter);
                    value = json_object_iter_value(iter);

                    if (strcmp(key, "position") == 0) {
                        json_t *data_json = json_object_get(json, "position");
                        if (data_json) {
		                    _observer_position = parse_vec3(data_json);
                        }
                    } else if (strcmp(key, "orientation") == 0) {
                        //data_json = json_object_get(json, "orientation");
                        //if (data_json) {
		                //    _observer_orientation = parse_quat(data_json);
                        //}
                    } else {
                        char *raw = json_dumps(value, 0);
                        _msgs.push(StimulusMessage(key,raw));
                        free(raw);
                    }

                    iter = json_object_iter_next(json, iter);
                }
                json_decref(json);
            } else {
                std::cerr << "JSON ERROR " << error.line << " " << error.text << "\n\n" << buf << "\n";
            }
        }
    }
};

class MyConnectionFactory : public Poco::Net::TCPServerConnectionFactory {
private:
    dsosg::DSOSG    *_dsosg;
    osg::Vec3       &_observer_position;
    osg::Quat       &_observer_orientation;
    Poco::Mutex     &_mutex;
    StimulusQueue   &_msgs;

public:
    MyConnectionFactory(dsosg::DSOSG *dsosg, osg::Vec3 &op, osg::Quat &oo, Poco::Mutex &m, StimulusQueue   &msgs) :
          _dsosg(dsosg),
          _observer_position(op),
          _observer_orientation(oo),
          _mutex(m),
          _msgs(msgs) {}

    virtual ~MyConnectionFactory() {}

    virtual Poco::Net::TCPServerConnection* createConnection(const Poco::Net::StreamSocket &socket)
    {
        return new MyConnection(socket, _dsosg, _observer_position, _observer_orientation, _mutex, _msgs);
    }
};

int main(int argc, char**argv) {
  dsosg::DSOSG *dsosg;
  Poco::Timestamp time;
  bool done;
  json_t *conf;
  json_error_t error;
  std::string display = "{}";
  Poco::Mutex mutex;
  StimulusQueue topicmessages;

  static const Poco::UInt16 SERVER_PORT = 8888;
  Poco::Net::ServerSocket sock(SERVER_PORT);

  osg::ArgumentParser arguments(&argc, argv);
  arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
  arguments.getApplicationUsage()->setDescription("Manual display/camera calibration utility");
  arguments.getApplicationUsage()->addCommandLineOption("--config <filename>","Display server config JSON file");
  arguments.getApplicationUsage()->addCommandLineOption("--display-mode <name>","Display mode");
  arguments.getApplicationUsage()->addCommandLineOption("--stimulus <name>","Name of stimulus plugin to start rendering");

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
  json_decref(conf);

  dsosg->setup_viewer("display_server", display);

  std::string stimulus = "Stimulus3DDemo";
  while(arguments.read("--stimulus", stimulus));
  dsosg->set_stimulus_plugin(stimulus);

  sock.listen();
  Poco::Net::TCPServer server(new MyConnectionFactory(
                                    dsosg,
                                    observer_position, observer_orientation,
                                    mutex,
                                    topicmessages),
                              sock);
  server.start();

  done = false;
  while (!done) {
    {
        Poco::Mutex::ScopedLock lock(mutex);
        float now;
        time.update();
        now = time.epochMicroseconds() * 1e6; /* microseconds to seconds */
        osg::Vec3 pos = observer_position;
        osg::Quat orr = observer_orientation;

        while (!topicmessages.empty()) {
            StimulusMessage &s = topicmessages.front();
            dsosg->topic_receive_json_message(s.topic, s.json);
            topicmessages.pop();
        }

        dsosg->update(now, pos, orr);
    }
 
    dsosg->frame();

    done = dsosg->done();
  }

  server.stop();
  sock.close();

  return 0;
}
