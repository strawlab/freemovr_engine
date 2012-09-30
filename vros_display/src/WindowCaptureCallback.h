#include <osg/Camera>
#include <osg/RenderInfo>

/** Capture the frame buffer and write image to disk*/
class WindowCaptureCallback : public osg::Camera::DrawCallback
{
public:
  WindowCaptureCallback(GLenum readBuffer, const std::string& name);
  virtual void operator () (osg::RenderInfo& renderInfo) const;
protected:
    GLenum                      _readBuffer;
    std::string                 _fileName;
    osg::ref_ptr<osg::Image>    _image;
    mutable OpenThreads::Mutex  _mutex;
};
