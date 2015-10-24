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
#include <osg/LightModel>

#include <osgGA/TrackballManipulator>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <osgViewer/ViewerEventHandlers>

#ifdef FLYVR_USE_CUDA
#include <osgCudaInit/Init> // ubuntu: apt-get install osgcompute
#endif

#include <stdio.h>
#include <stdexcept>

#include "Poco/Exception.h"
#include "Poco/Path.h"
#include "Poco/File.h"

#include <jansson.h>

#include "util.h"
#include "DisplaySurfaceGeometry.hpp"
#include "ProjectCubemapToGeometryPass.h"
#include "TexturedGeometryToCameraImagePass.h"
#include "CameraImageToDisplayImagePass.h"
#include "GeometryTextureToDisplayImagePass.h"

#include "flyvr/flyvr_assert.h"
#include "flyvr/ResourceLoader.hpp"
#include "flyvr/CallbackHolder.hpp"

// Notes:
//    Front face culling for dome projection:
//        http://www.mail-archive.com/osg-users@openscenegraph.net/msg01256.html

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
	CameraCube(osg::Node* input_node, osg::Node* observer_node, Poco::Path shader_path, unsigned int tex_width, unsigned int tex_height) {
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
        camera->setClearColor(osg::Vec4(0.5f, 0.0f, 0.0f, 0.0f)); // clear red

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

struct BackgroundCallback : public flyvr::BackgroundColorCallback {
public:
    BackgroundCallback(CameraCube* cc) : _camera_cube(cc) {}
    void setBackgroundColorImplementation(const osg::Vec4& color) const {
        _camera_cube->set_clear_color( color );
    }
private:
    CameraCube* _camera_cube;
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

osg::Group* ShowCubemap(osg::TextureCubeMap* texture, Poco::Path shader_path){
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

	ShowCubemapVertObj->loadShaderSourceFromFile( shader_path.absolute().append("show_cubemap.vert").toString());
	ShowCubemapFragObj->loadShaderSourceFromFile( shader_path.absolute().append("show_cubemap.frag").toString());

	osg::Uniform* observerViewCubeUniformSampler = new osg::Uniform( osg::Uniform::SAMPLER_CUBE, "observerViewCube" );

	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setAttributeAndModes(ShowCubemapProgram, osg::StateAttribute::ON);
	ss->addUniform( observerViewCubeUniformSampler );
	ss->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
	ss->setMode(GL_BLEND, osg::StateAttribute::ON);

	return group;
}

// constructor
DSOSG::DSOSG(std::string flyvr_basepath, std::string mode, float observer_radius,
             std::string config_fname, bool two_pass, bool show_geom_coords,
             bool tethered_mode, bool slave, unsigned int cubemap_resolution) :
    _current_stimulus(NULL), _mode(mode),
    _flyvr_basepath(flyvr_basepath),
    _config_file_path(config_fname),
    _tethered_mode(tethered_mode), _wcc(NULL), _g2di(NULL)

{
    json_error_t json_error;
    json_t *json_config;
    json_t *json_stimulus, *json_geom;
    bool ignore_missing_plugins = false;

    // ensure we interpret this as a directory (ensure trailing slash)
    _flyvr_basepath.makeAbsolute(); _flyvr_basepath.makeDirectory();
    // ensure we interpret this as a file
    _config_file_path.makeAbsolute(); _config_file_path.makeFile();

    Poco::Path shader_path(_flyvr_basepath);
    shader_path.pushDirectory("src"); shader_path.pushDirectory("shaders");

	// Check the mode is valid.
	if (!(_mode==std::string("cubemap") || _mode==std::string("vr_display") ||
		  _mode==std::string("virtual_world") || _mode==std::string("overview") ||
		  _mode==std::string("geometry") ||
          _mode==std::string("geometry_texture"))) {
		throw std::invalid_argument("unknown mode");
	}

    _observer_cb = new ObserverPositionCallback();

	osg::ref_ptr<osg::Group> root = new osg::Group; root->addDescription("root node");
	osg::Camera* debug_hud_cam = createHUD();
	root->addChild( debug_hud_cam );

    osg::LightModel* lightmodel = new osg::LightModel;
    lightmodel->setLocalViewer(true);
    root->getOrCreateStateSet()->setAttributeAndModes(lightmodel, osg::StateAttribute::ON);

	_hud_cam = create_HUD_cam(512,512);
	root->addChild( _hud_cam );

	std::string geom_filename;
	std::vector<std::string> stimulus_plugin_paths;
	std::vector<std::string> stimulus_plugin_names;

	std::string config_data_dir = _config_file_path.parent().toString();

	// Add the flyvr defaut stimulus plugins
    Poco::Path default_plugin_path = _flyvr_basepath;
    default_plugin_path.pushDirectory("lib");
	std::string default_lib_dir = default_plugin_path.toString();

	stimulus_plugin_paths.push_back(default_lib_dir); stimulus_plugin_names.push_back("Stimulus3DDemo");
	stimulus_plugin_paths.push_back(default_lib_dir); stimulus_plugin_names.push_back("Stimulus3DShaderDemo");
	stimulus_plugin_paths.push_back(default_lib_dir); stimulus_plugin_names.push_back("Stimulus2DBlit");
	stimulus_plugin_paths.push_back(default_lib_dir); stimulus_plugin_names.push_back("StimulusStandby");
	stimulus_plugin_paths.push_back(default_lib_dir); stimulus_plugin_names.push_back("StimulusOSG");

	if (!Poco::File(_config_file_path).exists()) {
		std::cerr << "configuration file " << _config_file_path.toString() << " does not exist." << std::endl;
		exit(1);
	}

    json_config = json_load_file(_config_file_path.toString().c_str(), 0, &json_error);
	json_stimulus = json_object_get(json_config, "stimulus_plugins");
	json_geom = json_object_get(json_config, "geom");

    _active_3d_world = new osg::Group; // each (3d) plugin switches the child of this node
    _observer_pat = new osg::PositionAttitudeTransform;
    _cubemap_maker = new CameraCube( _active_3d_world, _observer_pat, shader_path, cubemap_resolution, cubemap_resolution);
    _bg_callback = new BackgroundCallback(_cubemap_maker);

    ignore_missing_plugins = json_is_true(json_object_get(json_config, "ignore_missing_stimulus_plugins"));

    if (!json_is_array(json_stimulus)) {
		std::cerr << "config file must contain a valid list of stimulus plugins\n";
		exit(1);
	} else {
        for (unsigned int i=0; i<json_array_size(json_stimulus); i++) {
            // TODO FIXME: make sure that JSON is valid ("path" and
            // "name" keys exist) and fail nicely if not.
            Poco::Path plugin_path = _config_file_path.parent().resolve(
                                        Poco::Path(json_string_value (
                                                    json_object_get (
                                                    json_array_get (json_stimulus, i), "path"))));
            std::string plugin_name = json_string_value (json_object_get (json_array_get (json_stimulus, i), "name"));
			stimulus_plugin_paths.push_back( plugin_path.toString() );
			stimulus_plugin_names.push_back( plugin_name );
        }
    }

	{
		// load the shared library for each plugin path
		for (unsigned int i=0; i<stimulus_plugin_paths.size(); i++) {
            std::string plugin_name = stimulus_plugin_names.at(i);
            Poco::Path path_parent(stimulus_plugin_paths.at(i));
            Poco::Path path_leaf("lib" + plugin_name + Poco::SharedLibrary::suffix()); // append .dll or .so

            Poco::Path path_lib = path_parent;
            path_lib.append(path_leaf);
            path_lib.makeAbsolute();
            std::string lib_name = path_lib.toString();

            if (Poco::File(path_lib).exists()) {
			    try {
				    _stimulus_loader.loadLibrary(lib_name);
			    }
			    catch (Poco::Exception& exc) {
				    std::cerr << "ERROR loading library (" << lib_name << "): " << exc.displayText() << std::endl;
				    throw;
			    }

                try {
                    _stimulus_plugins[ plugin_name ] = _stimulus_loader.create(plugin_name);
                } catch (Poco::Exception& exc) {
				    std::cerr << "ERROR loading plugin from file " << lib_name << ": " << plugin_name << ": " << exc.displayText() << std::endl;
				    throw;
                } catch (...) {
                    std::cerr << "ERROR loading plugin from file " << lib_name << ": " << plugin_name << std::endl;
                    throw;
                }

				_stimulus_plugins[ plugin_name ]->set_flyvr_base_path(_flyvr_basepath.toString());
				_stimulus_plugins[ plugin_name ]->set_plugin_path(path_parent.toString());

                try {
                    _stimulus_plugins[ plugin_name ]->post_init(slave);
                } catch (...) {
                    std::cerr << "ERROR while calling post_init() on plugin: " << plugin_name << std::endl;
                    throw;
                }

                try {
                    _stimulus_plugins[ plugin_name ]->set_background_color_callback(_bg_callback);
                } catch (...) {
                    std::cerr << "ERROR while calling register_background_color_changed_callback() on plugin: " << plugin_name << std::endl;
                    throw;
                }

                // ensure we always have a current stimulus
				if (_current_stimulus == NULL) {
					_current_stimulus = _stimulus_plugins[ plugin_name ];
				}

            } else {
                std::cerr << "ERROR missing stimulus plugin: " << lib_name << std::endl;
                if (!ignore_missing_plugins)
                    throw std::runtime_error("Missing stimulus plugin " + lib_name);
            }
		}
	}

    // but default the current stimulus to the 3D one
	if (_stimulus_plugins.count("Stimulus3DDemo")) {
		// default is demo (only if present)
		_current_stimulus = _stimulus_plugins["Stimulus3DDemo"];
	}

	flyvr_assert(_current_stimulus != NULL);
	std::cout << "current stimulus name: " << _current_stimulus->name() << std::endl;

	_active_3d_world->addChild( _current_stimulus->get_3d_world() );

	_active_2d_hud = new osg::Group; // each (2d) plugin switches the child of this node
	_active_2d_hud->addChild( _current_stimulus->get_2d_hud() );
	_hud_cam->addChild(_active_2d_hud);

	if ( _mode==std::string("virtual_world") || _mode==std::string("overview")) {
		root->addChild( _active_3d_world );
	}

    _observer_pat->addDescription("observer position attitute transform node");
    _observer_pat->setPosition(osg::Vec3(0.0f,0.0f,0.0f));
    _observer_pat->setAttitude(osg::Quat(osg::inDegrees(0.0f),osg::Vec3(0.0f,0.0f,1.0f)));
    _observer_pat->setDataVariance(osg::Object::DYNAMIC);
    if (_mode==std::string("overview") || _mode==std::string("virtual_world") ||
        _mode==std::string("geometry")) {
		if (observer_radius != 0.0f) {
            _observer_marker_pat = new osg::PositionAttitudeTransform;
            _observer_marker_pat->setPosition(osg::Vec3(0.0f,0.0f,0.0f));
            osg::Quat attitude;
            attitude.makeRotate( osg::PI/2.0, 0, 1, 0);
            _observer_marker_pat->setAttitude(attitude);

            osg::Shape *shape;
            if (observer_radius > 0) {
                // draw a small red cone as the observer
                float height = 2* observer_radius;
                shape = new osg::Cone(osg::Vec3(0, 0, 0), observer_radius, height);
            } else {
                // draw a small red sphere as the observer
                shape = new osg::Sphere(osg::Vec3(0, 0, 0), -1.0*observer_radius);
            }

            osg::ref_ptr<osg::ShapeDrawable> shaped = new osg::ShapeDrawable(shape);
            osg::Geode* geode_1 = new osg::Geode;
            shaped->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
            osg::StateSet* ss = geode_1->getOrCreateStateSet();
            ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
            geode_1->addDrawable(shaped);
            _observer_marker_pat->addChild(geode_1);
            _observer_pat->addChild(_observer_marker_pat);
            root->addChild(_observer_pat);
		}
    }

	if ( !(_mode==std::string("virtual_world"))) {
		root->addChild(_cubemap_maker->get_node());
	}

    if (_mode==std::string("cubemap")) {
		// create On Screen Display for debugging
		osg::ref_ptr<osg::Group> debug_osd = ShowCubemap(_cubemap_maker->get_cubemap(),shader_path);
		root->addChild(debug_osd.get());
    }

    if (!json_is_object(json_geom)) {
		std::cerr << "config file must contain a valid projector geometry\n";
		exit(1);
	}

    DisplaySurfaceGeometry* geometry_parameters = new DisplaySurfaceGeometry(json_geom);
    _observer_geometry_pat = new osg::PositionAttitudeTransform;
    _observer_geometry_pat->addDescription("_observer_geometry_pat");
    if (_tethered_mode) {
        _observer_geometry_pat->setDataVariance(osg::Object::DYNAMIC);
    } else {
        _observer_geometry_pat->setPosition(osg::Vec3(0.0,0.0,0.0));
    }
    root->addChild( _observer_geometry_pat );

	// render cubemap onto geometry
	ProjectCubemapToGeometryPass *pctcp =new ProjectCubemapToGeometryPass(_flyvr_basepath.toString(),
																		  _cubemap_maker->get_cubemap(),
																		  _observer_cb,
																		  geometry_parameters);
	root->addChild(pctcp->get_top().get());
	if (_mode==std::string("overview")||_mode==std::string("geometry")) {
        // XXX should make the geometry itself move in tethered mode?
		_observer_geometry_pat->addChild( pctcp->get_textured_geometry() );
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

    if (_mode==std::string("overview")||_mode==std::string("vr_display")) {

        if (two_pass) {
            CameraModel* cam1_params = make_real_camera_parameters();
            osg::ref_ptr<osg::Group> g;
            if (show_geom_coords) {
                throw std::runtime_error("have not implemented this again");
                //g = geometry_parameters->make_texcoord_group(shader_path);
            } else {
                g = pctcp->get_textured_geometry();
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
                                                   1.0, 1.0,
                                                   0, 0, 0.3, 0.3);
                debug_hud_cam->addChild(g);
            }


            Poco::Path p2c_path = _config_file_path.parent().resolve(
                                     Poco::Path(json_string_value (
                                       json_object_get(json_config, "p2c"))));
            std::string p2c_filename = p2c_path.toString();
            std::cerr << "p2c file: " << p2c_filename << "\n";
            CameraImageToDisplayImagePass *ci2di = new CameraImageToDisplayImagePass(shader_path,
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
                                                       1.0, 1.0,
                                                       l,b,w,h);
                    debug_hud_cam->addChild(g);
                }
            }
        } else {

            json_t *p2g_json = json_object_get(json_config, "p2g");
            flyvr_assert(p2g_json != NULL);

            Poco::Path p2g_path = _config_file_path.parent().resolve(
                                    Poco::Path(json_string_value(p2g_json)));
            std::string p2g_filename = p2g_path.toString();
            std::cerr << "p2g file: " << p2g_filename << "\n";
            _g2di = new GeometryTextureToDisplayImagePass(shader_path,
                                                          pctcp->get_output_texture(),
                                                          p2g_filename,
                                                          show_geom_coords);
            root->addChild(_g2di->get_top().get());
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
                    osg::Group* g = make_textured_quad(_g2di->get_output_texture(),
                                                       -1.0,
                                                       1.0, 1.0,
                                                       0.0,0.0,w,h);
                    debug_hud_cam->addChild(g);
                }
            }
        }
    }

    _viewer = new osgViewer::Viewer;
    _viewer->setSceneData(root.get());
    _viewer->getViewerStats()->collectStats("frame_rate",true);

    json_decref(json_config);

    //osgDB::writeNodeFile(*(root.get()), _mode + std::string(".osg"));
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
    if (!_stimulus_plugins.count(name) == 1) {
        std::cerr << "ERROR: Could not find plugin when attempting to use: " << name << std::endl;
        std::cerr << "  available plugins are: " << std::endl;
        for (std::map<std::string, StimulusInterface*>::const_iterator i=_stimulus_plugins.begin();
             i!=_stimulus_plugins.end(); ++i ) {
            std::cerr << "    " << i->first << std::endl;
        }
        throw std::runtime_error("Requested stimulus was not found. Check stderr for more info.");
    }

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
    flyvr_assert(stimulus!=NULL);
	return stimulus->get_topic_names();
}

std::string DSOSG::stimulus_get_message_type(const std::string& plugin_name, const std::string& topic_name) {
    StimulusInterface* stimulus = _stimulus_plugins[plugin_name];
    flyvr_assert(stimulus!=NULL);
    try {
        return stimulus->get_message_type( topic_name );
    } catch (...) {
        std::cerr << "exception while calling stimulus->get_message_type(\"" << topic_name << "\")" << std::endl;
        throw; // rethrow the original exception
    }
}

void DSOSG::stimulus_receive_json_message(const std::string& plugin_name, const std::string& topic_name, const std::string& json_message) {
    StimulusInterface* stimulus = _stimulus_plugins[plugin_name];
    flyvr_assert(stimulus!=NULL);
	stimulus->receive_json_message( topic_name, json_message );
}

void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

std::string escape_filename(const std::string& fname) {
    std::string result;
    result = fname;
    replaceAll(result,"/","");
    return result;
}

void DSOSG::setup_viewer(const std::string& viewer_window_name, const std::string& json_config, bool use_pbuffer) {
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

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

		json_decref(root);
	}
	traits->width = width;
	traits->height = height;

    traits->alpha = 8;

    traits->pbuffer = use_pbuffer;

    osg::ref_ptr<osg::GraphicsContext> pbuffer;

    if (use_pbuffer) {
        pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());
        flyvr_assert(pbuffer.valid());
    }

    _viewer->addEventHandler(new osgViewer::ScreenCaptureHandler(
       new osgViewer::ScreenCaptureHandler::WriteToFile(
           std::string("screenshot_")+escape_filename(viewer_window_name),"png")));

    if (_mode==std::string("cubemap") || _mode==std::string("overview") ||
        _mode==std::string("geometry") ||
		_mode==std::string("geometry_texture") || _mode==std::string("virtual_world") ||
        use_pbuffer) {

        if (!use_pbuffer) {
            // setup in windowed mode

            if (_mode==std::string("cubemap") || _mode==std::string("geometry_texture")) {
                // best aspect ratio is square for this
                width=height;
            }

            // construct the viewer.
            _viewer->setUpViewInWindow( 32, 32, width, height );
        }

        _viewer->getCamera()->setProjectionMatrixAsPerspective(60.,
                                                               (double)width/(double)height,
                                                               0.01, 1000.);

        if (_mode==std::string("cubemap")) {
            _viewer->getCamera()->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f)); // clear black
        }

        if (_mode==std::string("overview") || _mode==std::string("virtual_world") ||
            _mode==std::string("geometry")) {
            _cameraManipulator = new osgGA::TrackballManipulator();
            _cameraManipulator->setAutoComputeHomePosition(true);
            _viewer->setCameraManipulator(_cameraManipulator);
        }
		_viewer->setReleaseContextAtEndOfFrameHint(false);

		_viewer->addEventHandler(new osgViewer::StatsHandler);

		osg::ref_ptr<osg::GraphicsContext> gc;
		gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		flyvr_assert_msg(gc.valid(),"could not create a graphics context with your desired traits");
		_viewer->getCamera()->setGraphicsContext(gc.get());
        _viewer->getCamera()->setClearColor(osg::Vec4(0.3f, 0.3f, 0.3f, 0.0f)); // clear dark gray

        _wcc = new WindowCaptureCallback();

        if (pbuffer.valid()) {
            osg::ref_ptr<osg::Camera> camera = new osg::Camera;
            camera->setGraphicsContext(pbuffer.get());
            camera->setViewport(new osg::Viewport(0,0,width,height));
            GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
            camera->setDrawBuffer(buffer);
            camera->setReadBuffer(buffer);
            camera->setFinalDrawCallback(_wcc);

            _viewer->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
        } else {
            _viewer->getCamera()->setFinalDrawCallback(_wcc);
        }

		_viewer->realize();
	}
	else if (_mode==std::string("vr_display")) {
		osg::ref_ptr<osg::GraphicsContext> gc;
		gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		flyvr_assert_msg(gc.valid(),"could not create a graphics context with your desired traits");
		_viewer->getCamera()->setGraphicsContext(gc.get());
		_viewer->getCamera()->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		_viewer->getCamera()->setDrawBuffer(buffer);
		_viewer->getCamera()->setReadBuffer(buffer);

        osg::ref_ptr<osg::GraphicsContext::ResizedCallback> rc = new DSOSGResizedCallback(gc->getResizedCallback(),this);
        gc->setResizedCallback(rc);

        setCursorVisible(false);

	}
	else {
		throw std::invalid_argument("unknown mode");
	}

    _viewer->setLightingMode( osg::View::SKY_LIGHT );

    // If the window frame is on, show the mouse cursor.
    if (traits->windowDecoration) {
        setCursorVisible(true);
    } else {
        setCursorVisible(false);
    }

    setWindowName(viewer_window_name);
	resized(width, height); // notify listeners that we have a new size

#ifdef FLYVR_USE_CUDA
    osgCuda::setupOsgCudaAndViewer( *_viewer );
#endif

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

        if (_observer_marker_pat.valid()) {
            // When not in tethered mode, we do not update any of the
            // display geometry position or orientation based on the
            // observer. However, in "overview", "virtual_world" or
            // "geometry" mode, we want to see the orientation of the
            // observer correctly.
            osg::Quat attitude;
            attitude.makeRotate( osg::PI/2.0, 0, 1, 0);
            attitude *= observer_orientation;
            _observer_marker_pat->setAttitude(attitude);
        }
    }

	if (_current_stimulus != NULL) {
        _current_stimulus->update( time, observer_position, observer_orientation );
    }
}

void DSOSG::frame() {
	_viewer->frame();
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_osg_capture_mutex);
    if (!_osg_capture_filename.empty()){
        osgDB::writeNodeFile(*(_active_3d_world.get()), _osg_capture_filename);
        _osg_capture_filename = "";
    }
};

bool DSOSG::done() {
	return _viewer->done();
};

float DSOSG::getFrameRate() {
    double framerate;
    _viewer->getViewerStats()->getAveragedAttribute("Frame rate", framerate, true);
    return framerate;
};

void DSOSG::setCursorVisible(bool visible) {
    osgViewer::Viewer::Windows windows;
    _viewer->getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end();++itr) {
        if (visible)
            (*itr)->setCursor(osgViewer::GraphicsWindow::InheritCursor);
        else
            (*itr)->setCursor(osgViewer::GraphicsWindow::NoCursor);
    }
};

void DSOSG::setWindowName(std::string name) {
    osgViewer::Viewer::Windows windows;
    _viewer->getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end();++itr) {
        (*itr)->setWindowName(name);
    }
};

void DSOSG::setCaptureImageFilename(std::string name) {
    flyvr_assert(_wcc!=NULL); // need to be in pbuffer or overview-type mode
    _wcc->set_next_filename( name );
}

void DSOSG::setCaptureOSGFilename(std::string name) {
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_osg_capture_mutex);
    _osg_capture_filename = name;
}

TrackballManipulatorState DSOSG::getTrackballManipulatorState() {
    flyvr_assert(_cameraManipulator.valid());
    TrackballManipulatorState result;
    result.rotation = _cameraManipulator->getRotation();
    result.center =  _cameraManipulator->getCenter();
    result.distance =  _cameraManipulator->getDistance();
    return result;
}

void DSOSG::setTrackballManipulatorState(TrackballManipulatorState s) {
    if (!_cameraManipulator.valid()) {
        // We could be rendering only the cubemap,
        // so we might not have camera manipulator.
		std::cerr << "ignoring request to set camera manipulator state." << std::endl;
        return;
    }
    _cameraManipulator->setRotation(s.rotation);
    _cameraManipulator->setCenter(s.center);
    _cameraManipulator->setDistance(s.distance);
}

void DSOSG::setGamma(float gamma) {
    if (_g2di)
        _g2di->set_gamma(gamma);
}

void DSOSG::setRedMax(bool red_max) {
    if (_g2di)
        _g2di->set_red_max(red_max);
}

bool DSOSG::is_CUDA_available() {
#ifdef FLYVR_USE_CUDA
  return true;
#else
  return false;
#endif
}

}

