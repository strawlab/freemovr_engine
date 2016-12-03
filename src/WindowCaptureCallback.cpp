#include "WindowCaptureCallback.h"
#include <iostream>
#include <osgDB/WriteFile>
#include "freemoovr_engine/freemoovr_assert.h"

WindowCaptureCallback::WindowCaptureCallback() : _gc(NULL) {
  _image = new osg::Image;
}

void WindowCaptureCallback::set_next_filename(const std::string& name) {
  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
  _fileName = name;
}

void WindowCaptureCallback::operator () (osg::RenderInfo& renderInfo) const {

  osg::GraphicsContext* gc = renderInfo.getState()->getGraphicsContext();
  if (_gc==NULL) {
    _gc=gc;
  }

  freemoovr_assert(_gc==gc); // only a single GraphicsContext supported

  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

  if (_fileName.empty()) return; // no point in capturing image if it's not used

  glReadBuffer(GL_BACK);
  const osg::GraphicsContext::Traits* traits = gc->getTraits();
  if (traits!=NULL) {
    GLenum pixelFormat;

    pixelFormat = GL_RGBA; // force saving alpha

    int width = traits->width;
    int height = traits->height;

    //std::cout<<"Capture: size="<<width<<"x"<<height<<", format="<<(pixelFormat == GL_RGBA ? "GL_RGBA":"GL_RGB")<<std::endl;

    _image->readPixels(0, 0, width, height, pixelFormat, GL_UNSIGNED_BYTE);

    //std::cout << "Writing to: " << _fileName << std::endl;
    osgDB::writeImageFile(*_image, _fileName);
    _fileName = "";
  }
}
