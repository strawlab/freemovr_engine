#include "ProjectCubemapToGeometryPass.h"

#include "Poco/Path.h"
#include "Poco/File.h"

#include <osgViewer/Viewer>
#include <osgDB/WriteFile>

#include "load_cubemap.h"
#include <fstream>
#include <iostream>

#include "util.h"
#include "vros_display/vros_assert.h"
#include "WindowCaptureCallback.h"

std::string get_file_contents(std::string filename) {
  // this is the simplest way in C++?
  std::ifstream myfile;
  std::string contents;
  std::string tmp;
  myfile.open(filename.c_str(),std::ios::in);
  while (!myfile.eof()) {
    myfile >> tmp;
    contents += tmp;
  }
  myfile.close();
  return contents;
}

int main(int argc, char**argv) {

  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0] << " CUBEMAP_DIR JSON_FILE OUTPUT_FILE" << std::endl;
    return -1;
  }

  Poco::Path cubemap_dir(argv[1]);
  Poco::Path json_file(argv[2]);
  Poco::Path output_file(argv[3]);

  Poco::Path src(__FILE__);
  Poco::Path vros_display_basepath = src.parent();
  vros_display_basepath.popDirectory(); // remove trailing /src directory
  std::cout << "vros_display_basepath: " << vros_display_basepath.toString() << std::endl;

  osg::ref_ptr<osg::TextureCubeMap> my_cubemap;
  my_cubemap = load_cubemap( cubemap_dir.toString(), "jpg" );

  std::string contents = get_file_contents( json_file.toString() );

  DisplaySurfaceGeometry* geometry_parameters= new DisplaySurfaceGeometry(contents.c_str());

  ProjectCubemapToGeometryPass *pctcp =new ProjectCubemapToGeometryPass( vros_display_basepath.toString(),
                                                                         my_cubemap,
                                                                         NULL,
                                                                         geometry_parameters);

  osg::ref_ptr<osg::Group> root = new osg::Group; root->addDescription("root node");
  osg::Camera* debug_hud_cam = createHUD();
  root->addChild( debug_hud_cam );

  root->addChild(pctcp->get_top().get());


  osg::Texture2D* geomtex = pctcp->get_output_texture();
  osg::Group* g = make_textured_quad(geomtex,
                                     -1.0,
                                     1.0,
                                     1.0,
                                     0, 0, 1.0, 1.0);
  debug_hud_cam->addChild(g);

  // ---- The rest of this just renders an offscreen frame and saves
  // ---- to a file. The bits at the end (multiple calls to frame(),
  // ---- etc.) seem necessary to avoid a seg fault.

  osgViewer::Viewer* _viewer = new osgViewer::Viewer;
  _viewer->setSceneData(root.get());

  osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
  traits->windowName = argv[0];
  traits->width = 512;
  traits->height = 512;
  traits->pbuffer = true;

  osg::ref_ptr<osg::GraphicsContext>gc = osg::GraphicsContext::createGraphicsContext(traits.get());
  vros_assert(gc.valid());
  _viewer->getCamera()->setGraphicsContext(gc.get());
  _viewer->getCamera()->setViewport(new osg::Viewport(0,0, traits->width, traits->height));

  _viewer->realize();
  _viewer->frame();

  osg::ref_ptr<WindowCaptureCallback> wcc = new WindowCaptureCallback();
  wcc->set_next_filename(output_file.toString());
  _viewer->getCamera()->setFinalDrawCallback(wcc);

  _viewer->frame();
  _viewer->renderingTraversals();

  _viewer->getCamera()->setFinalDrawCallback(NULL);
  _viewer->frame();

  std::cout << "deleting" << std::endl;
  std::cout << "all done" << std::endl;
}
