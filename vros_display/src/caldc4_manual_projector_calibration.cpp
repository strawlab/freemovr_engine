/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include <OpenThreads/ScopedLock>

#include <osg/ArgumentParser>
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
    int run();
private:
    void setup_viewer(std::string json_config, int &width, int &height);
    osgViewer::Viewer* _viewer;
    osg::PositionAttitudeTransform * pat;
    int _height;
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

MyNode::MyNode(int argc, char**argv)
{
    Poco::Path exe_path(argv[0]); exe_path.makeFile();
    Poco::Path image_path(exe_path.makeParent());

    osg::ArgumentParser arguments(&argc, argv);
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->addCommandLineOption("--cfg <filename>","Display config JSON file");

	std::string json_filename = "display-config.json";
    while(arguments.read("--cfg", json_filename));

    std::string json_message;
    {
        std::ifstream f;
        f.open( json_filename.c_str() );

        std::getline( f, json_message ); // XXX This will fail when a newline is in the file!
        f.close();
    }

	osg::ref_ptr<osg::Group> root = new osg::Group; root->addDescription("root node");

    _viewer = new osgViewer::Viewer;
    _viewer->setSceneData(root.get());

	// construct the viewer.
    int width,height;
    setup_viewer(json_message, width,height);
    {
        osgViewer::Viewer::Windows windows;
        _viewer->getWindows(windows);
        for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end();++itr)
            {
                //(*itr)->useCursor(false);
                (*itr)->setCursor(osgViewer::GraphicsWindow::NoCursor);
            }

    }
    _viewer->addEventHandler(new KeyboardEventHandler(this));

    // set up the texture state.
    if (Poco::File(image_path).isFile()) {
        std::cerr << "Could not read my image file: " << image_path.toString() << std::endl;
        exit(1);
    }

	osg::Image* image = osgDB::readImageFile( image_path.toString() );
	if (!image) {
		throw std::runtime_error("Could not open image file");
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

void MyNode::mouse_move( int _mx, int _my ) {
    pat->setPosition(osg::Vec3(_mx,_my,0.0f));
}

void MyNode::mouse_click( int _mx, int _my ) {
    std::cout << _mx << " " << _height-_my << std::endl;
    pat->setPosition(osg::Vec3(_mx,_my,0.0f));
}

int MyNode::run() {
    while (!_viewer->done()) {
        _viewer->frame();


    }
    return 0;
}

void MyNode::setup_viewer(std::string json_config, int& width, int& height) {
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = "display server";

	{
		json_t *root;
		json_error_t error;

		root = json_loads(json_config.c_str(), 0, &error);
		if(!root) {
			fprintf(stderr, "error: in %s(%d) on json line %d: %s\n", __FILE__, __LINE__, error.line, error.text);
            throw std::runtime_error("Could not read JSON");
		}

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
			traits->x = json_integer_value( win_origin_x_json );
		}

		json_t *win_origin_y_json = json_object_get(root, "y");
		if(json_is_integer(win_origin_y_json)){
			traits->y = json_integer_value( win_origin_y_json );
		}

		json_t *tmp_json;
		tmp_json = json_object_get(root, "hostName");
		if (json_is_string(tmp_json)) {
			traits->hostName = json_string_value( tmp_json );
		}

		tmp_json = json_object_get(root, "displayNum");
		if (json_is_integer(tmp_json)) {
			traits->displayNum = json_integer_value( tmp_json );
		}

		tmp_json = json_object_get(root, "screenNum");
		if (json_is_integer(tmp_json)) {
			traits->screenNum = json_integer_value( tmp_json );
		}

		traits->windowDecoration = false;
		traits->overrideRedirect = true;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->pbuffer = false;

		json_decref(root);
	}
	traits->width = width;
	traits->height = height;

    {
		osg::ref_ptr<osg::GraphicsContext> gc;
		gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		assert(gc.valid());

		//_viewer->setUpViewInWindow( win_origin_x, win_origin_y, width, height );
		_viewer->getCamera()->setGraphicsContext(gc.get());
		_viewer->getCamera()->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
        _height= traits->height;
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		_viewer->getCamera()->setDrawBuffer(buffer);
		_viewer->getCamera()->setReadBuffer(buffer);

	}
    //	resized(width, height); // notify listeners that we have a new size
}


int main(int argc, char**argv) {
	MyNode* n=new MyNode(argc,argv);
	return n->run();
}
