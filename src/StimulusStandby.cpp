/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "freemovr_engine/StimulusInterface.hpp"
#include "util.h"

#include "Poco/ClassLibrary.h"

#include <iostream>
#include <stdexcept>

#include <osg/Texture2D>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

class StimulusStandby: public StimulusInterface
{
public:
StimulusStandby(std::string package_share_dir) : StimulusInterface(package_share_dir) {
}

std::string name() const {
    return "StimulusStandby";
}

virtual void post_init(bool slave) {
    std::string fname = get_plugin_data_path(std::string("/Pond/posz.jpg"));
    osg::Image* image = osgDB::readImageFile(fname);
    osg::Texture2D* texture = new osg::Texture2D(image);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR );
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    _hud = make_textured_quad(texture,
							  -1.0,
							  1.0, 1.0,
							  0.0, 0, 1.0, 1.0);
}

void resized(int width,int height) {
}

osg::ref_ptr<osg::Group> get_3d_world() {
    return 0;
}

osg::ref_ptr<osg::Group> get_2d_hud() {
    return _hud;
}

std::vector<std::string> get_topic_names() const {
	std::vector<std::string> result;
	return result;
}

std::string get_message_type(const std::string& topic_name) const {
	throw std::runtime_error("no message types to get");
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
}

private:
	osg::ref_ptr<osg::Group> _hud;
};

MAKE_STIMULUS_INTERFACE_LOADER(StimulusStandby);

POCO_BEGIN_MANIFEST(StimulusInterfaceLoader)
POCO_EXPORT_CLASS(StimulusStandbyLoader)
POCO_END_MANIFEST

// optional set up and clean up functions
void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
