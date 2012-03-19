/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "dsosg.h"

#include <OpenThreads/ScopedLock>

#include <osg/Projection>
#include <osg/Geometry>
#include <osg/Texture>
#include <osg/TexGen>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PolygonOffset>
#include <osg/TextureCubeMap>
#include <osg/TexMat>
#include <osg/ClearNode>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/ArgumentParser>
#include <osg/TextureRectangle>
#include <osg/Texture2D>
#include <osg/Camera>
#include <osg/TexGenNode>
#include <osg/View>

#include <osgGA/TrackballManipulator>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <osgViewer/ViewerEventHandlers>

#include <assert.h>

#include <stdio.h>
#include <stdexcept>

#include "Poco/AutoPtr.h"
#include "Poco/Util/XMLConfiguration.h"
#include "Poco/Manifest.h"
#include "Poco/Exception.h"

#include <boost/filesystem.hpp>

#include <jansson.h>

#include "util.h"
#include "display_screen_geometry.h"
#include "camera_model.h"
#include "ProjectCubemapToGeometryPass.h"
#include "TexturedGeometryToCameraImagePass.h"
#include "CameraImageToDisplayImagePass.h"
#include "GeometryTextureToDisplayImagePass.h"

// Notes:
//    Front face culling for dome projection:
//        http://www.mail-archive.com/osg-users@openscenegraph.net/msg01256.html

typedef Poco::Manifest<StimulusInterface> StimulusManifest;

namespace dsosg
{


typedef std::vector< osg::ref_ptr<osg::Camera> >  CameraList;

class UpdateCameraCallback : public osg::NodeCallback
{
public:
    UpdateCameraCallback(osg::Node *observerNode, CameraList& Cameras):
            _Cameras(Cameras)
        {
          _observerNodePath.push_back(observerNode);
        }

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            // first update subgraph to make sure objects are all moved into position
            traverse(node,nv);

            // compute the position of the center of the reflector subgraph
            osg::Matrixd worldToLocal = osg::computeWorldToLocal(_observerNodePath);

            typedef std::pair<osg::Vec3, osg::Vec3> ImageData;
            const ImageData id[] =
            {
                ImageData( osg::Vec3( 1,  0,  0), osg::Vec3( 0, -1,  0) ), // +X
                ImageData( osg::Vec3(-1,  0,  0), osg::Vec3( 0, -1,  0) ), // -X
                ImageData( osg::Vec3( 0,  1,  0), osg::Vec3( 0,  0,  1) ), // +Y
                ImageData( osg::Vec3( 0, -1,  0), osg::Vec3( 0,  0, -1) ), // -Y
                ImageData( osg::Vec3( 0,  0,  1), osg::Vec3( 0, -1,  0) ), // +Z
                ImageData( osg::Vec3( 0,  0, -1), osg::Vec3( 0, -1,  0) )  // -Z
            };

            for(unsigned int i=0;
                i<6 && i<_Cameras.size();
                ++i)
            {
                osg::Matrix localOffset;
                localOffset.makeLookAt(osg::Vec3(0.0,0.0,0.0), id[i].first, id[i].second);

                osg::Matrix viewMatrix = worldToLocal*localOffset;

                _Cameras[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
                _Cameras[i]->setProjectionMatrixAsFrustum(-1.0,1.0,-1.0,1.0,1.0,10000.0);
                _Cameras[i]->setViewMatrix(viewMatrix);
            }
        }

    protected:

        virtual ~UpdateCameraCallback() {}

        osg::NodePath               _observerNodePath;
        CameraList                  _Cameras;
};

osg::ref_ptr<osg::Group>add_bg_quad(std::string shader_dir, std::string fname) {
    osg::Image* image = osgDB::readImageFile(fname);
	osg::Texture2D* texture = new osg::Texture2D(image);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR );
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

	osg::Group* group = new osg::Group;

	osg::Geode* geode = new osg::Geode();
	{
		// make quad
		osg::Vec2Array* vertices = new osg::Vec2Array;
		float low=-1.0;
		vertices->push_back(  osg::Vec2(low, low) );
		vertices->push_back(  osg::Vec2(low,  1.0) );
		vertices->push_back(  osg::Vec2( 1.0,  1.0) );
		vertices->push_back(  osg::Vec2( 1.0, low) );

		osg::ref_ptr<osg::Geometry> this_geom = new osg::Geometry();
		this_geom->setVertexArray(vertices);
		this_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
		geode->addDrawable(this_geom);
	}
	group->addChild(geode);

	std::vector< osg::ref_ptr<osg::Program> > _programList;
	osg::Program* BackgroundImageProgram;
	osg::Shader*  BackgroundImageVertObj;
	osg::Shader*  BackgroundImageFragObj;

	BackgroundImageProgram = new osg::Program;
	BackgroundImageProgram->setName( "background_image" );
	_programList.push_back( BackgroundImageProgram );
	BackgroundImageVertObj = new osg::Shader( osg::Shader::VERTEX );
	BackgroundImageFragObj = new osg::Shader( osg::Shader::FRAGMENT );
	BackgroundImageProgram->addShader( BackgroundImageFragObj );
	BackgroundImageProgram->addShader( BackgroundImageVertObj );

	LoadShaderSource( BackgroundImageVertObj, join_path(shader_dir,"background_image.vert" ));
	LoadShaderSource( BackgroundImageFragObj, join_path(shader_dir, "background_image.frag" ));

	osg::Uniform* observerViewCubeUniformSampler = new osg::Uniform( osg::Uniform::SAMPLER_2D, "BackgroundImage" );

	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setAttributeAndModes(BackgroundImageProgram, osg::StateAttribute::ON);
	ss->addUniform( observerViewCubeUniformSampler );
	ss->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
	//ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderBinDetails(-1,"RenderBin");
	ss->setMode(GL_DEPTH, osg::StateAttribute::OFF);

	return group;
}

osg::ref_ptr<osg::Camera> create_HUD_cam(unsigned int width, unsigned int height)
{
  // create a camera to set up the projection and model view matrices, and the subgraph to drawn in the HUD
  osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,width,0,height));

    // set the view matrix
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::POST_RENDER);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    camera->setAllowEventFocus(false);

    return camera;
}

struct DSOSGResizedCallback : public osg::GraphicsContext::ResizedCallback {
public:
	DSOSGResizedCallback(osg::ref_ptr<osg::GraphicsContext::ResizedCallback> orig_cb, DSOSG* dsosg) : _orig_cb(orig_cb), _dsosg(dsosg) {}
	void resizedImplementation(osg::GraphicsContext* gc, int x, int y, int width, int height) {
		_dsosg->resized( width, height );
		if (_orig_cb.valid()) {
			_orig_cb->resizedImplementation(gc, x, y, width, height);
		} else {
			gc->resizedImplementation(x, y, width, height);
		}
	}
private:
	osg::ref_ptr<osg::GraphicsContext::ResizedCallback> _orig_cb;
	DSOSG* _dsosg;
};

class CameraCube {
public:
	CameraCube(osg::Node* input_node, osg::Node* observer_node, std::string config_data_dir, std::string shader_dir, unsigned int tex_width=512, unsigned int tex_height=512) {
    _texture = new osg::TextureCubeMap;
    _top = new osg::Group; _top->addDescription("CameraCube top node");

    _texture->setTextureSize(tex_width, tex_height);

    _texture->setInternalFormat(GL_RGBA);
    _texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    _texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    _texture->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
    _texture->setFilter(osg::TextureCubeMap::MIN_FILTER,osg::TextureCubeMap::LINEAR);
    _texture->setFilter(osg::TextureCubeMap::MAG_FILTER,osg::TextureCubeMap::LINEAR);

    // set up the render to texture cameras.
    for(unsigned int i=0; i<6; ++i)
    {
        // create the camera
        osg::Camera* camera = new osg::Camera;

        camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        camera->setClearColor(osg::Vec4(0.5f, 0.0f, 0.0f, 1.0f)); // clear

        // set viewport
        camera->setViewport(0,0,tex_width,tex_height);

        // set the camera to render before the main camera.
        camera->setRenderOrder(osg::Camera::PRE_RENDER);

        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, _texture, 0, i);

        // add subgraph to render
        camera->addChild(input_node);
        _top->addChild(camera);

        Cameras.push_back(camera);
    }

    // set an update callback to keep moving the camera and tex gen in the right direction.
    _top->setUpdateCallback(new UpdateCameraCallback(observer_node, Cameras));
  }
  void set_clear_color( const osg::Vec4& color) {
    for(unsigned int i=0; i<6; ++i)
    {
        osg::Camera* camera = Cameras.at(i);
        camera->setClearColor(color);
    }
      
  }
  osg::Node* get_node() {
    return _top.get();
  }
  osg::TextureCubeMap* get_cubemap() {
    return _texture.get();
  }
private:
  osg::ref_ptr<osg::TextureCubeMap> _texture;
  osg::ref_ptr<osg::Group> _top;
  CameraList Cameras;
};


void ObserverPositionCallback::operator() ( osg::Uniform* uniform, osg::NodeVisitor* nv )
{
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
	uniform->set( _p );
}

void ObserverPositionCallback::setObserverPosition( osg::Vec3 p ) {
	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
	_p = p;
}

osg::Group* ShowCubemap(osg::TextureCubeMap* texture,std::string shader_dir){
	osg::Group* group = new osg::Group;

	osg::Geode* geode = new osg::Geode();
	{
		// make quad
		osg::Vec2Array* vertices = new osg::Vec2Array;
		float low=-1.0;
		vertices->push_back(  osg::Vec2(low, low) );
		vertices->push_back(  osg::Vec2(low,  1.0) );
		vertices->push_back(  osg::Vec2( 1.0,  1.0) );
		vertices->push_back(  osg::Vec2( 1.0, low) );

		osg::ref_ptr<osg::Geometry> this_geom = new osg::Geometry();
		this_geom->setVertexArray(vertices);
		this_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
		geode->addDrawable(this_geom);
	}
	group->addChild(geode);

	std::vector< osg::ref_ptr<osg::Program> > _programList;
	osg::Program* ShowCubemapProgram;
	osg::Shader*  ShowCubemapVertObj;
	osg::Shader*  ShowCubemapFragObj;

	ShowCubemapProgram = new osg::Program;
	ShowCubemapProgram->setName( "show_cubemap" );
	_programList.push_back( ShowCubemapProgram );
	ShowCubemapVertObj = new osg::Shader( osg::Shader::VERTEX );
	ShowCubemapFragObj = new osg::Shader( osg::Shader::FRAGMENT );
	ShowCubemapProgram->addShader( ShowCubemapFragObj );
	ShowCubemapProgram->addShader( ShowCubemapVertObj );

	LoadShaderSource( ShowCubemapVertObj, join_path(shader_dir,"show_cubemap.vert" ));
	LoadShaderSource( ShowCubemapFragObj, join_path(shader_dir, "show_cubemap.frag" ));

	osg::Uniform* observerViewCubeUniformSampler = new osg::Uniform( osg::Uniform::SAMPLER_CUBE, "observerViewCube" );

	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setAttributeAndModes(ShowCubemapProgram, osg::StateAttribute::ON);
	ss->addUniform( observerViewCubeUniformSampler );
	ss->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
	ss->setMode(GL_BLEND, osg::StateAttribute::ON);

	return group;
}

// constructor
DSOSG::DSOSG(std::string vros_display_basepath, std::string mode, float observer_radius, std::string config_fname, bool two_pass, bool show_geom_coords) : _current_stimulus(NULL), _mode(mode), _vros_display_basepath(vros_display_basepath), _tethered_mode(true)
{
	std::string _shader_dir = (_vros_display_basepath/"src"/"shaders").string();

	// Check the mode is valid.
	if (!(_mode==std::string("cubemap") || _mode==std::string("vr_display") ||
		  _mode==std::string("virtual_world") || _mode==std::string("overview") ||
		  _mode==std::string("geometry_texture"))) {
		throw std::invalid_argument("unknown mode");
	}

    _observer_cb = new ObserverPositionCallback();

	osg::ref_ptr<osg::Group> root = new osg::Group; root->addDescription("root node");
	osg::Camera* debug_hud_cam = createHUD();
	root->addChild( debug_hud_cam );


	_hud_cam = create_HUD_cam(512,512);
	root->addChild( _hud_cam );

	using Poco::AutoPtr;
	using Poco::Util::AbstractConfiguration;
	using Poco::Util::XMLConfiguration;

	namespace fs = boost::filesystem;

	std::string geom_filename;
	std::vector<std::string> stimulus_plugin_paths;
	std::vector<std::string> stimulus_plugin_names;
	std::string config_data_dir;

	{
		// Add the vros_display defaut stimulus plugins
		std::string default_path = (_vros_display_basepath/"lib").string();
		stimulus_plugin_paths.push_back(default_path); stimulus_plugin_names.push_back("stimulus_3d_demo");
		stimulus_plugin_paths.push_back(default_path); stimulus_plugin_names.push_back("stimulus_2d_blit");
		stimulus_plugin_paths.push_back(default_path); stimulus_plugin_names.push_back("stimulus_standby");
	}

	{
		// get all the plugin paths
		fs::path config_full_path(config_fname);
#if BOOST_FILESYSTEM_VERSION >= 3
		fs::path config_dir = boost::filesystem3::complete(config_full_path.parent_path()).normalize();
#else
		fs::path config_dir = config_full_path.parent_path();
#endif
		config_data_dir = config_dir.string();

		if (!fs::exists(config_full_path)) {
			std::cerr << "configuration file " << config_full_path.string() << " does not exist." << std::endl;
			exit(1);
		}
		AutoPtr<XMLConfiguration> pConf(new XMLConfiguration(config_full_path.string()));

		{
			AutoPtr<AbstractConfiguration> stimulus_plugin_config = pConf->createView("stimulus_plugins");
			AbstractConfiguration::Keys k;
			stimulus_plugin_config->keys(k);
			for (AbstractConfiguration::Keys::const_iterator ki=k.begin(); ki!=k.end(); ++ki) {
				std::string pathkey = std::string((*ki)+std::string("[@path]"));
				std::string namekey = std::string((*ki)+std::string("[@name]"));
				fs::path plugin_path = stimulus_plugin_config->getString(pathkey); // this is relative to config_dir
				if (plugin_path.has_relative_path()) {
#if BOOST_FILESYSTEM_VERSION >= 3
					plugin_path = fs::absolute(plugin_path,config_dir).normalize();
#else
                    std::cerr << "warning: not implemented: relative plugin path, but continuing anyway" << std::endl;
#endif
				}
                std::cout << "plugin_path: " << plugin_path << std::endl;

				std::string plugin_name = stimulus_plugin_config->getString(namekey);
				stimulus_plugin_paths.push_back( plugin_path.string() );
				stimulus_plugin_names.push_back( plugin_name );
			}
		}

		std::string geom_path_raw = pConf->getString("display_geometry[@path]");
		fs::path geom_path;
#if BOOST_FILESYSTEM_VERSION >= 3
		geom_path = fs::absolute(geom_path_raw,config_dir).normalize();
#else
        geom_path = config_dir / geom_path_raw;
#endif
		if (!fs::exists(geom_path)) {
			std::cerr << "geometry file " << geom_path.string() << " does not exist." << std::endl;
			exit(1);
		}
		geom_filename = geom_path.string();

	}

	{
		// load the shared library for each plugin path
		for (unsigned int i=0; i<stimulus_plugin_paths.size(); i++) {
			fs::path path_parent = stimulus_plugin_paths.at(i);
			fs::path path_leaf = "lib" + stimulus_plugin_names.at(i) + Poco::SharedLibrary::suffix(); // append .dll or .so

			std::string libName = (path_parent/path_leaf).normalize().string();

			try {
				_stimulus_loader.loadLibrary(libName);
			}
			catch (Poco::Exception& exc) {
				std::cerr << exc.displayText() << std::endl;
				exc.rethrow();
			}

		}
	}

	{
        std::cout << "Now loading plugins. If you get seg faults at this point, make sure " << \
            "your plugins are compiled against the latest stimulus_interface.h" << std::endl;
        // XXX Should implement some kind of version checking to prevent such seg faults...

		// iterate over each library's plugins
		StimulusLoader::Iterator it(_stimulus_loader.begin());
		StimulusLoader::Iterator end(_stimulus_loader.end());
		for (; it != end; ++it) {
			StimulusManifest::Iterator itMan(it->second->begin());
			StimulusManifest::Iterator endMan(it->second->end());
			for (; itMan != endMan; ++itMan) {
				assert( _stimulus_plugins.count( itMan->name() ) == 0);
				_stimulus_plugins[ itMan->name() ] = itMan->create();
				_stimulus_plugins[ itMan->name() ]->set_vros_display_base_path(_vros_display_basepath.string());
                try {
                    _stimulus_plugins[ itMan->name() ]->post_init(config_data_dir);
                } catch (...) {
                    std::cerr << "ERROR: while calling post_init() on plugin " << itMan->name() << std::endl;
                    throw std::runtime_error("error while calling post_init()");
                }
				if (_current_stimulus == NULL) {
					_current_stimulus = _stimulus_plugins[ itMan->name() ];
				}
			}
		}
	}

	if (_stimulus_plugins.count("Stimulus3DDemo")) {
		// default is demo (only if present)
		_current_stimulus = _stimulus_plugins["Stimulus3DDemo"];
	}

	assert(_current_stimulus != NULL);
	std::cout << "current stimulus name: " << _current_stimulus->name() << std::endl;

	_active_3d_world = new osg::Group; // each (3d) plugin switches the child of this node
	_active_3d_world->addChild( _current_stimulus->get_3d_world() );

	_active_2d_hud = new osg::Group; // each (2d) plugin switches the child of this node
	_active_2d_hud->addChild( _current_stimulus->get_2d_hud() );
	_hud_cam->addChild(_active_2d_hud);

	if ( _mode==std::string("virtual_world") || _mode==std::string("overview")) {
		root->addChild( _active_3d_world );
	}

    _observer_pat = new osg::PositionAttitudeTransform;
    _observer_pat->addDescription("observer position attitute transform node");
    _observer_pat->setPosition(osg::Vec3(0.0f,0.0f,0.0f));
    _observer_pat->setAttitude(osg::Quat(osg::inDegrees(0.0f),osg::Vec3(0.0f,0.0f,1.0f)));
    _observer_pat->setDataVariance(osg::Object::DYNAMIC);
    if (_mode==std::string("overview") || _mode==std::string("virtual_world")) {
		if (observer_radius != 0.0f) {
			// draw a small red sphere as the observer
			osg::Geode* geode_1 = new osg::Geode;
			osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),
																							observer_radius));
			shape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
			osg::StateSet* ss = geode_1->getOrCreateStateSet();
			ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
			geode_1->addDrawable(shape);
			_observer_pat->addChild(geode_1);
			root->addChild(_observer_pat);
		}
    }

	_cubemap_maker = new CameraCube( _active_3d_world, _observer_pat, config_data_dir, _shader_dir );

	if ( !(_mode==std::string("virtual_world"))) {
		root->addChild(_cubemap_maker->get_node());
	}

    if (_mode==std::string("cubemap")) {
		// create On Screen Display for debugging
		osg::ref_ptr<osg::Group> debug_osd = ShowCubemap(_cubemap_maker->get_cubemap(),_shader_dir);
		root->addChild(debug_osd.get());
    }

	std::cout << "reading display geometry from " << geom_filename << std::endl;
	DisplaySurfaceGeometry* geometry_parameters = new DisplaySurfaceGeometry( geom_filename );
    _observer_geometry_pat = new osg::PositionAttitudeTransform;
    _observer_geometry_pat->setDataVariance(osg::Object::DYNAMIC);
    root->addChild( _observer_geometry_pat );

	// render cubemap onto geometry
	ProjectCubemapToGeometryPass *pctcp =new ProjectCubemapToGeometryPass(_shader_dir,
																		  _cubemap_maker->get_cubemap(),
																		  _observer_cb,
																		  geometry_parameters);
	root->addChild(pctcp->get_top().get());
	if (_mode==std::string("overview")) {
        // XXX should make the geometry itself move in tethered mode?
		_observer_geometry_pat->addChild( pctcp->get_textured_geometry() );
	}

	if (two_pass) {
		CameraModel* cam1_params = make_real_camera_parameters();
		osg::ref_ptr<osg::Group> g;
		if (show_geom_coords) {
			throw std::runtime_error("have not implemented this again");
			//g = geometry_parameters->make_texcoord_group(_shader_dir);
		} else {
			g = pctcp->get_textured_geometry();
		}

		if (_mode==std::string("geometry_texture")) {
            osg::Texture2D* geomtex = pctcp->get_output_texture();

			osg::Group* g = make_textured_quad(geomtex,
											   -1.0,
											   1.0,
											   1.0,
											   0, 0, 1.0, 1.0);
			debug_hud_cam->addChild(g);

		}

		TexturedGeometryToCameraImagePass *tg2ci=new TexturedGeometryToCameraImagePass(g,
																					   cam1_params);
		root->addChild(tg2ci->get_top().get());
		osg::Texture* mytex;
		mytex = tg2ci->get_output_texture();
		if (_mode==std::string("overview")) {
			root->addChild( cam1_params->make_rendering(1) );
			osg::Group* g = make_textured_quad(mytex,
											   -1.0,
											   cam1_params->width(),
											   cam1_params->height(),
											   0, 0, 0.3, 0.3);
			debug_hud_cam->addChild(g);
		}

		std::string p2c_filename = join_path(config_data_dir,"p2c.exr" );
		CameraImageToDisplayImagePass *ci2di = new CameraImageToDisplayImagePass(_shader_dir,
																				 mytex,
																				 p2c_filename);
		root->addChild(ci2di->get_top().get());
		{
			bool show_hud = false;
			float l,b,w,h;
			l=0.0; b=0.0; w=1.0; h=1.0;
			if (_mode==std::string("overview")) {
				show_hud=true;
				l=0.3; b=0.0; w=0.3; h=0.3;
			}
			if (_mode==std::string("vr_display")) {
				show_hud=true;
			}
			if (show_hud) {
				osg::Group* g = make_textured_quad(ci2di->get_output_texture(),
												   -1.0,
												   ci2di->get_display_width(), ci2di->get_display_height(),
												   l,b,w,h);
				debug_hud_cam->addChild(g);
			}
		}
	} else {
		std::string p2g_filename = join_path(config_data_dir,"p2g.exr" );
        std::cout << "loading p2g.exr from " << p2g_filename << std::endl;
		GeometryTextureToDisplayImagePass *g2di = new GeometryTextureToDisplayImagePass(_shader_dir,
																						pctcp->get_output_texture(),
																						p2g_filename,
																						show_geom_coords);
		root->addChild(g2di->get_top().get());
		{
			bool show_hud = false;
			float w,h;
			w=1.0; h=1.0;
			if (_mode==std::string("overview")) {
				show_hud=true;
				w=0.3; h=0.3;
			}
			if (_mode==std::string("vr_display")) {
				show_hud=true;
			}
			if (show_hud) {
				osg::Group* g = make_textured_quad(g2di->get_output_texture(),
												   -1.0,
												   g2di->get_display_width(), g2di->get_display_height(),
												   0.0,0.0,w,h);
				debug_hud_cam->addChild(g);
			}
		}
	}


    _viewer = new osgViewer::Viewer;
    _viewer->setSceneData(root.get());
    _viewer->setLightingMode(osg::View::NO_LIGHT);
}

std::vector<std::string> DSOSG::get_stimulus_plugin_names() {
	std::vector<std::string> result;
	for (std::map<std::string, StimulusInterface*>::const_iterator i=_stimulus_plugins.begin();
		 i!=_stimulus_plugins.end(); ++i ) {
		result.push_back( i->first );
	}
	return result;
}

void DSOSG::set_stimulus_plugin(const std::string& name) {
	std::cout << "setting stimulus plugin: " << name << std::endl;
	assert( _stimulus_plugins.count(name) == 1);
	// switch off old stimulus
	_active_3d_world->removeChild( _current_stimulus->get_3d_world() );
	_active_2d_hud->removeChild( _current_stimulus->get_2d_hud() );

	// switch on new stimulus
	_current_stimulus = _stimulus_plugins[ name ];
	_active_3d_world->addChild( _current_stimulus->get_3d_world() );
	_active_2d_hud->addChild( _current_stimulus->get_2d_hud() );

    // set the background color
    _cubemap_maker->set_clear_color( _current_stimulus->get_clear_color() );

    // It we have an interactive camera manipulator, it means we can reset the view.
    if (_cameraManipulator.valid())
    {
        // This will compute a new home position. (Press space to get there.)
        _cameraManipulator->setNode(_current_stimulus->get_3d_world());
    }

}

std::vector<std::string> DSOSG::stimulus_get_topic_names(const std::string& plugin_name) {
    StimulusInterface* stimulus = _stimulus_plugins[plugin_name];
    assert(stimulus!=NULL);
	return stimulus->get_topic_names();
}

std::string DSOSG::stimulus_get_message_type(const std::string& plugin_name, const std::string& topic_name) {
    StimulusInterface* stimulus = _stimulus_plugins[plugin_name];
    assert(stimulus!=NULL);
    try {
        return stimulus->get_message_type( topic_name );
    } catch (...) {
        std::cerr << "exception while calling stimulus->get_message_type(\"" << topic_name << "\")" << std::endl;
        throw; // rethrow the original exception
    }
}

void DSOSG::stimulus_receive_json_message(const std::string& plugin_name, const std::string& topic_name, const std::string& json_message) {
    StimulusInterface* stimulus = _stimulus_plugins[plugin_name];
    assert(stimulus!=NULL);
	stimulus->receive_json_message( topic_name, json_message );
}

void DSOSG::setup_viewer(const std::string& json_config) {
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = "display server";

    // make sure these have decent defaults
	int width = 512;
	int height = 512;

	{
		json_t *root;
		json_error_t error;

		root = json_loads(json_config.c_str(), 0, &error);
		if(!root) {
			fprintf(stderr, "error: in %s(%d) on json line %d: %s\n", __FILE__, __LINE__, error.line, error.text);
			throw std::runtime_error("error in json");
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
		tmp_json = json_object_get(root, "windowDecoration");
		if (json_is_true(tmp_json)) {
            // inorder to add window redirections the redered scene must go through the window
            // manager / X11, it cannot just blit to the display. This has two implications
            // 1) things might be slower
            // 2) absolute positioning of opengl windows might not work because the window
            //    manager can move them around.
			traits->windowDecoration = true;
			traits->overrideRedirect = false;
		} else if (json_is_false(tmp_json)) {
			traits->windowDecoration = false;
			traits->overrideRedirect = true;
		}
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->pbuffer = false;

		json_decref(root);
	}
	traits->width = width;
	traits->height = height;

    if (_mode==std::string("cubemap") || _mode==std::string("overview") ||
		_mode==std::string("geometry_texture") || _mode==std::string("virtual_world")) {

        if (1) {
            // setup in windowed mode with this resolution
            width = 750;
            height = 550;

            if (_mode==std::string("cubemap") || _mode==std::string("geometry_texture")) {
                // best aspect ratio is square for this
                width=height;
            }


            // construct the viewer.
            _viewer->setUpViewInWindow( 32, 32, width, height );

            _viewer->getCamera()->setProjectionMatrixAsPerspective(60.,
                                                                   (double)width/(double)height,
                                                                   0.01, 1000.);
        }
        if (_mode==std::string("cubemap")) {
            _viewer->getCamera()->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f)); // black
        }

        if (_mode==std::string("overview") || _mode==std::string("virtual_world")) {
            _cameraManipulator = new osgGA::TrackballManipulator();
            _cameraManipulator->setAutoComputeHomePosition(true);
            _viewer->setCameraManipulator(_cameraManipulator);
        }
		_viewer->setReleaseContextAtEndOfFrameHint(false);

		_viewer->addEventHandler(new osgViewer::StatsHandler);
		_viewer->realize();
	}
	else if (_mode==std::string("vr_display")) {
		osg::ref_ptr<osg::GraphicsContext> gc;
		gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		assert(gc.valid());

		//_viewer->setUpViewInWindow( win_origin_x, win_origin_y, width, height );
		_viewer->getCamera()->setGraphicsContext(gc.get());
		_viewer->getCamera()->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		_viewer->getCamera()->setDrawBuffer(buffer);
		_viewer->getCamera()->setReadBuffer(buffer);

		{
			osg::GraphicsContext* gc = _viewer->getCamera()->getGraphicsContext();
			osg::ref_ptr<osg::GraphicsContext::ResizedCallback> rc = new DSOSGResizedCallback(gc->getResizedCallback(),this);
			gc->setResizedCallback(rc);
		}

	}
	else {
		throw std::invalid_argument("unknown mode");
	}

	resized(width, height); // notify listeners that we have a new size
};

void DSOSG::resized(const int& width, const int& height) {
	_width = width;
	_height = height;
	{
		_hud_cam->setProjectionMatrix(osg::Matrix::ortho2D(0,width,0,height));
	}

	for (std::map<std::string, StimulusInterface*>::iterator i=_stimulus_plugins.begin();
		 i!=_stimulus_plugins.end(); ++i ) {
		i->second->resized(width,height);
	}

}

void DSOSG::update( const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation ) {
    // ignoring orientation for now...
	_observer_pat->setPosition(observer_position); // update the location of the cameras that project onto cubemap
    if (_tethered_mode) {
        _observer_pat->setAttitude(observer_orientation); // rotate the cameras that project onto cubemap
        // we fix the observer at location (0,0,0) for the purposes of rendering the texture on the geometry
        _observer_cb->setObserverPosition(osg::Vec3(0.0,0.0,0.0)); // update the shader that projects cubemap onto geometry
        _observer_geometry_pat->setPosition(observer_position); // update the display rendering in various debug modes (e.g. overview mode)
    } else {
        // we let the observer move within the geometry
        _observer_pat->setAttitude(osg::Quat()); // do not rotate the cameras that project onto cubemap
        _observer_cb->setObserverPosition(observer_position); // update the shader that projects cubemap onto geometry
        _observer_geometry_pat->setPosition(osg::Vec3(0.0,0.0,0.0)); // update the display rendering in various debug modes (e.g. overview mode)
    }

	if (_current_stimulus != NULL) {
        _current_stimulus->update( time, observer_position, observer_orientation );
    }
}

void DSOSG::frame() {
	_viewer->frame();
};

bool DSOSG::done() {
	return _viewer->done();
};

}
