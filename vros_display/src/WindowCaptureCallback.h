// -*- Mode: C++ -*-
#include <osg/Camera>
#include <osg/RenderInfo>

/** Capture the frame buffer and write image to disk*/
class WindowCaptureCallback : public osg::Camera::DrawCallback
{
public:

  WindowCaptureCallback();

  virtual void operator () (osg::RenderInfo& renderInfo) const;
  virtual void set_next_filename(const std::string& name);

  osg::ref_ptr<osg::Image>    _image;
  mutable OpenThreads::Mutex  _mutex;
  mutable osg::GraphicsContext* _gc;
  mutable std::string _fileName;
};
