/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#ifndef DSOSG_H
#define DSOSG_H

#include <osg/Vec3>
#include <osg/PositionAttitudeTransform>
#include <osg/Uniform>

#include <osgViewer/Viewer>

#include <boost/filesystem.hpp>

#include "Poco/ClassLoader.h"
#include "Poco/Manifest.h"
#include "vros_display/stimulus_interface.h"

namespace dsosg{

	typedef Poco::ClassLoader<StimulusInterface> StimulusLoader;

	class ObserverPositionCallback: public osg::Uniform::Callback {
	public:
		ObserverPositionCallback() {}
		virtual void operator() ( osg::Uniform* uniform, osg::NodeVisitor* nv );
		virtual void setObserverPosition( osg::Vec3 p );
	private:
		OpenThreads::Mutex      _mutex;
		osg::Vec3 _p;
	};

	class DSOSG {
	public:
		DSOSG(std::string vros_display_basepath, std::string mode, float observer_radius, std::string config_fname, bool two_pass=false, bool show_geom_coords=false);

		std::vector<std::string> get_stimulus_plugin_names();
		std::string get_current_stimulus_plugin_name() { return std::string(_current_stimulus->name()); }
		void set_stimulus_plugin(std::string name);

		std::vector<std::string> current_stimulus_get_topic_names();
		std::string current_stimulus_get_message_type(std::string topic_name);
		void current_stimulus_send_json_message(std::string topic_name, std::string json_message);

		void setup_viewer(std::string json_config);
		void resized(int width, int height);

		void set_observer_pose( osg::Vec3 observer_pose);
		void frame();
		bool done();

		int getXSize() {return _width;}
		int getYSize() {return _height;}
	private:
		StimulusLoader _stimulus_loader;

		std::map<std::string, StimulusInterface*> _stimulus_plugins;
		StimulusInterface* _current_stimulus;
		std::string _mode;
		boost::filesystem::path _vros_display_basepath;
		osgViewer::Viewer* _viewer;
		osg::ref_ptr<osg::PositionAttitudeTransform> _observer_pat;
		ObserverPositionCallback* _observer_cb;

		osg::ref_ptr<osg::Camera> _hud_cam;
		osg::ref_ptr<osg::Group> _active_3d_world;
		osg::ref_ptr<osg::Group> _active_2d_hud;

		int _width;
		int _height;
	};

}

#endif
