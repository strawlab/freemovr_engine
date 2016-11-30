/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#include "freemoovr/StimulusInterface.hpp"
#include "util.h"
#include "json2osg.hpp"
#include "base64.h"

#include "Poco/ClassLibrary.h"
#include "Poco/NumberFormatter.h"

#include <sstream>
#include <cstdio>

#include <osg/TextureRectangle>
#include <osg/Texture2D>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <osgText/Font>
#include <osgText/Text>

#include <stdexcept>

#include <jansson.h>

#include "freemoovr/freemoovr_assert.h"

class StimulusLatencyTimestamp: public StimulusInterface
{
public:
    StimulusLatencyTimestamp() { _rootg = new osg::Group; };

    void post_init(bool);
    void resized(int width,int height);
    void update(const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation);

    std::vector<std::string> get_topic_names() const { return std::vector<std::string>(); }
    std::string get_message_type(const std::string& topic_name) const { throw std::runtime_error("no message types to get"); }
    std::string name() const { return "StimulusLatencyTimestamp"; }
    void receive_json_message(const std::string&, const std::string&) {}
    osg::ref_ptr<osg::Group> get_3d_world() { return 0; }
    osg::ref_ptr<osg::Group> get_2d_hud() { return _rootg; }
    osg::Vec4 get_clear_color() const { return osg::Vec4(1.0, 1.0, 1.0, 1.0); } //white
private:

    osg::ref_ptr<osg::Group> _rootg;
    osg::ref_ptr<osg::Group> _group;
    osg::ref_ptr<osg::Geode> _geode;
    osg::ref_ptr<osgText::Text> _text;
    osg::ref_ptr<osg::Geometry> _bg_geom;
};

void StimulusLatencyTimestamp::resized(int width,int height) {
    _text->setCharacterSize(width/10.0);
    _text->setPosition(osg::Vec3(width/2.0,height/2.0,0.0f));

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array;
    vertices->push_back(  osg::Vec3(0.0,  0.0, -1.0 )); texcoords->push_back(osg::Vec2(0.0, 0.0));
    vertices->push_back(  osg::Vec3(0.0, height, -1.0 )); texcoords->push_back(osg::Vec2(0.0, height));
    vertices->push_back( osg::Vec3(width, height, -1.0 )); texcoords->push_back(osg::Vec2(width, height));
    vertices->push_back( osg::Vec3(width,  0.0, -1.0 )); texcoords->push_back(osg::Vec2(width, 0.0));
    _bg_geom->setVertexArray(vertices);
    _bg_geom->setTexCoordArray(0,texcoords);
}

void StimulusLatencyTimestamp::update(const double& time, const osg::Vec3& observer_position, const osg::Quat& observer_orientation) {
    std::string s = Poco::NumberFormatter::format(time);
    _text->setText("*"+s+"*");
}

void StimulusLatencyTimestamp::post_init(bool) {

    _group = new osg::Group;
    osg::StateSet* ss = _group->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    _rootg->addChild(_group);
    _group->addDescription("textured quad group");

    std::string s = get_plugin_data_path("free3of9.ttf");
    osgText::Font* font = osgText::readFontFile(s);

    bool useVBOs = false;

    osg::Vec4 layoutColor(0.0f,0.0f,0.0f,1.0f); //black

    _geode = new osg::Geode();
    _text = new osgText::Text();
    _text->setUseVertexBufferObjects(useVBOs);
    _text->setFont(font);
    _text->setFontResolution(200,200);
    _text->setColor(layoutColor);
    _text->setCharacterSize(20.0);
    _text->setPosition(osg::Vec3(0.0f,0.0f,0.0f));  //this is updated in resized
    _text->setAlignment(osgText::Text::CENTER_CENTER);

    _text->setText("*ABC123*");
    _geode->addDrawable(_text);

    //create a white background fullscreen quad (the size is set in resize)
    _bg_geom = new osg::Geometry();
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    _bg_geom->setColorArray(colors);
    _bg_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    _bg_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
    _geode->addDrawable(_bg_geom);

    _group->addChild(_geode);

}

POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusLatencyTimestamp)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}

