/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#include <osg/Group>
#include <osgViewer/Viewer>
#include <osgDB/WriteFile>

#include "util.h"
#include "exrutil.h"
#include "display_screen_geometry.h"
#include "camera_model.h"

#include "TexturedGeometryToCameraImagePass.h"
#include "CameraImageToDisplayImagePass.h"

int main(int argc, char *argv[])
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer* viewer = new osgViewer::Viewer();
    viewer->setUpViewInWindow( 32, 32, 1024, 768 );

    std::string _shader_dir = "shaders";
    std::string config_data_dir = "../config";

    osg::Group* root = new osg::Group;

	CameraImageToDisplayImagePass *ci2di;

    {
		DisplaySurfaceGeometry* geometry_parameters = new DisplaySurfaceGeometry( "geom.json" );
		CameraModel* cam1_params = make_real_camera_parameters();

		osg::ref_ptr<osg::Group> geom_group = new osg::Group;
		{
			osg::ref_ptr<osg::Geometry> this_geom = geometry_parameters->make_geom();
			osg::ref_ptr<osg::Geode> geode = new osg::Geode;
			osg::StateSet* _state_set = this_geom->getOrCreateStateSet();
			_state_set->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
			geode->addDrawable(this_geom.get());
			geom_group->addChild(geode.get());
		}

		TexturedGeometryToCameraImagePass *tg2ci=new TexturedGeometryToCameraImagePass(geom_group,
																					   cam1_params);
		root->addChild(tg2ci->get_top().get());

		std::string p2c_filename = join_path(config_data_dir,"p2c.exr" );
		bool UseHDR=false;
		ci2di = new CameraImageToDisplayImagePass(_shader_dir,
												  tg2ci->get_output_texture(),
												  p2c_filename,
												  UseHDR);
		root->addChild(ci2di->get_top().get());
    }

	osg::ref_ptr<osg::Image> p2g_image = new osg::Image;
	if (1) {
		osg::Camera* image_camera = new osg::Camera;

		// set the projection matrix
		image_camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1.0,0,1.0));

		// set the view matrix
		image_camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		image_camera->setViewMatrix(osg::Matrix::identity());

		// draw subgraph after main camera view.
		image_camera->setRenderOrder(osg::Camera::POST_RENDER);

		// we don't want the camera to grab event focus from the viewers main camera(s).
		image_camera->setAllowEventFocus(false);

		image_camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		p2g_image->allocateImage(ci2di->get_display_width(), ci2di->get_display_height(), 1, GL_RGBA, GL_FLOAT);
		//p2g_image->allocateImage(ci2di->get_display_width(), ci2di->get_display_height(), 1, GL_RGB, GL_UNSIGNED_BYTE);

		image_camera->setViewport(0, 0, ci2di->get_display_width(), ci2di->get_display_height());
		image_camera->attach(osg::Camera::COLOR_BUFFER, p2g_image.get());

		osg::Group* g = make_textured_quad(ci2di->get_output_texture(),
										   -1.0,
										   ci2di->get_display_width(), ci2di->get_display_height(),
										   0,0,1,1);
		image_camera->addChild( g );

		root->addChild( image_camera );
	}

    viewer->setSceneData(root);
    viewer->realize();

    viewer->frame(); // render a single frame

	save_exr("p2g.exr",p2g_image.get() );

    return 0;
}
