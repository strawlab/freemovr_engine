/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include <OpenThreads/ScopedLock>

#include <osg/ArgumentParser>
#include <osg/ApplicationUsage>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/Projection>
#include <osg/Geometry>
#include <osg/Texture>
#include <osg/TexGen>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PolygonOffset>
#include <osg/CullSettings>
#include <osg/TextureCubeMap>
#include <osg/TexMat>
#include <osg/MatrixTransform>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/PolygonOffset>
#include <osg/CullFace>
#include <osg/Material>
#include <osg/PositionAttitudeTransform>
#include <osg/ArgumentParser>
#include <osg/TextureRectangle>
#include <osg/Texture2D>
#include <osg/Camera>
#include <osg/TexGenNode>
#include <osg/View>
#include <osg/io_utils>
#include <osg/AutoTransform>

#include <osgGA/TrackballManipulator>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <osgViewer/ViewerEventHandlers>

#include "Poco/Path.h"
#include "Poco/File.h"

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <jansson.h>

#include <assert.h>
#include <stdio.h>
#include <stdexcept>
#include <sstream>
#include <iostream>

// forward
class MyNode {
public:
    MyNode(int argc, char**argv);
    void mouse_move( int _mx, int _my );
    void mouse_click( int _mx, int _my );
    void joy_callback(const sensor_msgs::JoyConstPtr& message);
    int run();
private:
    void setup_viewer_json(std::string json_config, int &width_out, int &height_out);
    void setup_viewer_ros(std::string &root, int &width_out, int &height_out);
    void setup_viewer(int &x, int &y, int &width, int &height, int &displayNum, int &screenNum);
    osgViewer::Viewer* _viewer;
    osg::PositionAttitudeTransform * pat;
    int _height;
    ros::NodeHandle* _node;
    ros::Subscriber _sub;
    float _joystick_factor;
};

class KeyboardEventHandler : public osgGA::GUIEventHandler
{
public:

    KeyboardEventHandler(MyNode* mn) : app(mn)
    {}

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
        switch (ea.getEventType())
        {
        case(osgGA::GUIEventAdapter::PUSH):
        case(osgGA::GUIEventAdapter::MOVE):
            {

                _mx = ea.getX();
                _my = ea.getY();

                app->mouse_move( _mx, _my );
                return false;
            }
        case(osgGA::GUIEventAdapter::RELEASE):
            {
                /*
             	if (_mx == ea.getX() && _my == ea.getY())
                    {
                        // only do if the mouse hasn't moved
                        app->mouse_click( _mx, _my );
                    }
                */
                _mx = ea.getX();
                _my = ea.getY();

                app->mouse_click( _mx, _my );

                return true;
            }

        default:
            break;

        }
        //return false to allow mouse manipulation
        return false;
    }

protected:

    float _mx,_my;
    MyNode* app;

};



osg::Camera* createBG(int width, int height)
{
    // create a camera to set up the projection and model view matrices, and the subgraph to drawn in the HUD
    osg::Camera* camera = new osg::Camera;
	camera->addDescription("background camera");

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,width,0,height));

    // set the view matrix
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
	camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	camera->setClearColor(osg::Vec4(0.0f, 0.1f, 0.0f, 1.0f)); // green

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::PRE_RENDER);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    camera->setAllowEventFocus(false);

	return camera;
}

MyNode::MyNode(int argc, char**argv) : 
    _joystick_factor(1.0)
{
    json_error_t json_error;
    json_t *json_config, *json_display;
    int width,height;

    std::vector<std::string> non_ros_args;
    ros::removeROSArgs (argc, argv, non_ros_args);

    osg::ArgumentParser arguments(&argc, argv);
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription("Manual display/camera calibration utility");
    arguments.getApplicationUsage()->addCommandLineOption("--config <filename>","Display server config JSON file");
    arguments.getApplicationUsage()->addCommandLineOption("--display-server <path>","Parameter server path to display_server");
    arguments.getApplicationUsage()->addCommandLineOption("--joystick-factor <number>","When using joystick for calibration, scales movement by this many pixels");

    osg::ApplicationUsage::Type help = arguments.readHelpType();
    if (help != osg::ApplicationUsage::NO_HELP) {
        arguments.getApplicationUsage()->write(std::cout);
        exit(0);
    }

	std::string json_filename = "";
    while(arguments.read("--config", json_filename));

	std::string display_server_path = "";
    while(arguments.read("--display-server", display_server_path));

    while(arguments.read("--joystick-factor", _joystick_factor));

    if (json_filename.empty() && display_server_path.empty()) {
        ROS_FATAL("must specify one of --config or --display-server");
        arguments.getApplicationUsage()->write(std::cout);
        exit(1);
    }

    // start ros
    ros::init(argc, argv, "manual_display_calibration");
    _node = new ros::NodeHandle();
    _sub = _node->subscribe("/joy", 10, &MyNode::joy_callback, this);

	// setup the viewer.
	osg::ref_ptr<osg::Group> root = new osg::Group; root->addDescription("root node");
    _viewer = new osgViewer::Viewer;
    _viewer->setSceneData(root.get());

    if (!json_filename.empty()) {
        if (!Poco::File(Poco::Path(json_filename)).isFile()) {
            ROS_FATAL("could not find json file");
            arguments.getApplicationUsage()->write(std::cout);
            exit(1);
        } else {
            ROS_INFO("configuring from JSON file %s", json_filename.c_str());
            json_config = json_load_file(json_filename.c_str(), 0, &json_error);
	        json_display = json_object_get(json_config, "display");
            std::string json_message = json_dumps(json_display, 0);
            setup_viewer_json(json_message, width, height);
        }
    } else {
        ROS_INFO("configuring from ROS file %s", json_filename.c_str());
        setup_viewer_ros(display_server_path, width, height);
    } 

    osgViewer::Viewer::Windows windows;
    _viewer->getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end();++itr) {
        (*itr)->setCursor(osgViewer::GraphicsWindow::NoCursor);
    }
    _viewer->addEventHandler(new KeyboardEventHandler(this));

    // set up the texture state.
    Poco::Path exe_path(argv[0]); exe_path.absolute().makeFile();
    Poco::Path image_path(exe_path.makeParent().makeParent());
    image_path.pushDirectory("data"); image_path.setFileName("cursor.png");

    if (!Poco::File(image_path).isFile()) {
        ROS_FATAL("could not read image file %s", image_path.toString().c_str());
        exit(1);
    }

	osg::Image* image = osgDB::readImageFile( image_path.toString() );
	if (!image) {
        ROS_FATAL("could not open image file %s", image_path.toString().c_str());
        exit(1);
	}

	osg::Camera* bgcam = createBG( width, height );
	root->addChild( bgcam );
	{
        pat = new osg::PositionAttitudeTransform;

		osg::Texture2D* texture = new osg::Texture2D(image);
		osg::Geode* geode = new osg::Geode;
		geode->addDescription("background texture geode");
		{

			osg::Vec3 pos = osg::Vec3(-64.0f,-64.0f,0.0f);
			osg::Vec3 width(128,0.0f,0.0);
			osg::Vec3 height(0.0,128,0.0);
			osg::Geometry* geometry = osg::createTexturedQuadGeometry(pos,width,height);
            geode->addDrawable(geometry);

			osg::StateSet* stateset = geode->getOrCreateStateSet();
			stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
			stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
			stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		}
        pat->addChild(geode);
		bgcam->addChild(pat);

	}

    _viewer->getCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	_viewer->getCamera()->setClearMask( GL_DEPTH_BUFFER_BIT);

}

void MyNode::joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
    float x = msg->axes[0];
    float y = msg->axes[1];

    if (msg->buttons[0]) {
        osg::Vec3 curr = pat->getPosition();
        std::cout << curr.x() << " " << _height-curr.y() << std::endl;
        return;
    }

    osg::Vec3 newpos = pat->getPosition() + osg::Vec3(-1.0*_joystick_factor*x,_joystick_factor*y,0.0);
    pat->setPosition(newpos);
}

void MyNode::mouse_move( int _mx, int _my ) {
    pat->setPosition(osg::Vec3(_mx,_my,0.0f));
}

void MyNode::mouse_click( int _mx, int _my ) {
    std::cout << _mx << " " << _height-_my << std::endl;
    pat->setPosition(osg::Vec3(_mx,_my,0.0f));
}

int MyNode::run() {
    while (!_viewer->done() && ros::ok()) {
        _viewer->frame();
        ros::spinOnce();
    }
    return 0;
}

void MyNode::setup_viewer(int &x, int &y, int &width, int &height, int &displayNum, int &screenNum) {
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = "configure projector";
    traits->width = width;
    traits->height = height;
	traits->x = x;
	traits->y = y;
	traits->hostName = '\0';
	traits->displayNum = displayNum;
	traits->screenNum = screenNum;

	traits->windowDecoration = false;
	traits->overrideRedirect = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->pbuffer = false;

	osg::ref_ptr<osg::GraphicsContext> gc;
	gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	assert(gc.valid());

	//_viewer->setUpViewInWindow( win_origin_x, win_origin_y, width, height );
	_viewer->getCamera()->setGraphicsContext(gc.get());
	_viewer->getCamera()->setViewport(new osg::Viewport(0,0, width, traits->height));
    _height= traits->height;
	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
	_viewer->getCamera()->setDrawBuffer(buffer);
	_viewer->getCamera()->setReadBuffer(buffer);
}

void MyNode::setup_viewer_json(std::string json_config, int& width, int& height) {
	json_t *root;
	json_error_t error;

	root = json_loads(json_config.c_str(), 0, &error);
	if(!root) {
        ROS_FATAL("error in json line %d: %s", error.line, error.text);
        throw std::runtime_error("Could not read JSON");
	}

    int x, y, displayNum, screenNum;

	json_t *width_json = json_object_get(root, "width");
	if(json_is_integer(width_json)){
		width = json_integer_value( width_json );
	}

	json_t *height_json = json_object_get(root, "height");
	if(json_is_integer(height_json)){
		height = json_integer_value( height_json );
	}

	json_t *win_origin_x_json = json_object_get(root, "x");
	if(json_is_integer(win_origin_x_json)){
		x = json_integer_value( win_origin_x_json );
	}

	json_t *win_origin_y_json = json_object_get(root, "y");
	if(json_is_integer(win_origin_y_json)){
		y = json_integer_value( win_origin_y_json );
	}

	json_t *tmp_json;
	tmp_json = json_object_get(root, "displayNum");
	if (json_is_integer(tmp_json)) {
		displayNum = json_integer_value( tmp_json );
	}

	tmp_json = json_object_get(root, "screenNum");
	if (json_is_integer(tmp_json)) {
		screenNum = json_integer_value( tmp_json );
	}

    setup_viewer(x, y, width, height, displayNum, screenNum);
}

void MyNode::setup_viewer_ros(std::string &root, int& width, int& height) {
    int x, y, displayNum, screenNum;

    ros::param::get (root + "/display/x", x);
    ros::param::get (root + "/display/y", y);
    ros::param::get (root + "/display/width", width);
    ros::param::get (root + "/display/height", height);
    ros::param::get (root + "/display/screenNum", screenNum);
    ros::param::get (root + "/display/displayNum", displayNum);

    setup_viewer(x, y, width, height, displayNum, screenNum);
}


int main(int argc, char**argv) {
	MyNode* n=new MyNode(argc,argv);
	return n->run();
}
