/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#ifndef DSOSG_H
#define DSOSG_H

#include <osg/Vec3>
#include <osg/PositionAttitudeTransform>
#include <osg/Uniform>

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

#include <jansson.h>

#include "Poco/ClassLoader.h"
#include "Poco/Manifest.h"
#include "Poco/Path.h"

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

	class CameraCube; // forward declaration

	class DSOSG {
	public:
		DSOSG(std::string vros_display_basepath, std::string mode, float observer_radius, std::string config_fname, bool two_pass=false, bool show_geom_coords=false);

		std::vector<std::string> get_stimulus_plugin_names();
		std::string get_current_stimulus_plugin_name() { return std::string(_current_stimulus->name()); }
		void set_stimulus_plugin(const std::string& plugin_name);

		std::vector<std::string> stimulus_get_topic_names(const std::string& plugin_name);
		std::string stimulus_get_message_type(const std::string& plugin_name, const std::string& topic_name);
		void stimulus_receive_json_message(const std::string& plugin_name, const std::string& topic_name, const std::string& json_message);

		void setup_viewer(const std::string& viewer_window_name, const std::string& json_config);
		void resized(const int& width, const int& height);

        void update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation );
		void frame();
		bool done();

		int getXSize() {return _width;}
		int getYSize() {return _height;}

		float getFrameRate();
		void setCursorVisible(bool visible);
	private:
		StimulusLoader _stimulus_loader;

		std::map<std::string, StimulusInterface*> _stimulus_plugins;
		StimulusInterface* _current_stimulus;
		std::string _mode;

		Poco::Path _vros_display_basepath;
		Poco::Path _config_file_path;

		osgViewer::Viewer* _viewer;
		osg::ref_ptr<osg::PositionAttitudeTransform> _observer_pat;
		osg::ref_ptr<osg::PositionAttitudeTransform> _observer_geometry_pat;
		ObserverPositionCallback* _observer_cb;
		bool _tethered_mode;

		CameraCube* _cubemap_maker;

		osg::ref_ptr<osg::Camera> _hud_cam;
		osg::ref_ptr<osg::Group> _active_3d_world;
		osg::ref_ptr<osg::Group> _active_2d_hud;

		int _width;
		int _height;
		osg::ref_ptr<osgGA::TrackballManipulator> _cameraManipulator;
	};

}

#endif
