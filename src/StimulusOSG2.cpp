#include "freemovr_engine/StimulusInterface.hpp"
#include "freemovr_engine/freemovr_assert.h"

#include "json2osg.hpp"

#include "Poco/ClassLibrary.h"
#include "Poco/Format.h"

#include <iostream>
#include <osg/io_utils>

#include <map>
#include <vector>

#include <osg/MatrixTransform>
#include <osg/TextureCubeMap>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/CullFace>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LightModel>
#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Bone>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

#include <jansson.h>

//#define _DEBUG

using Poco::format;

class StringUtils
{
protected:
    StringUtils() {}
    ~StringUtils() {}

public:
    static StringUtils* instance();

    typedef std::vector<std::string>					Tokens;
    typedef std::vector<std::string>::iterator			TokensIterator;
    typedef std::vector<std::string>::const_iterator	TokensConstIterator;

    Tokens tokenize(const std::string& str, const std::string& delimiters = " ")
    {
        Tokens tokens;
        std::string::size_type delimPos = 0, tokenPos = 0, pos = 0;

        if(str.length()<1)  return tokens;
        while(1)
        {
            delimPos = str.find_first_of(delimiters, pos);
            tokenPos = str.find_first_not_of(delimiters, pos);
            if (tokenPos != std::string::npos && str[tokenPos]=='\"')
            {
                delimPos = str.find_first_of("\"", tokenPos+1);
                pos++;
            }

            if(std::string::npos != delimPos)
            {
                if(std::string::npos != tokenPos)
                {
                    if(tokenPos<delimPos)
                    {
                        std::string token = str.substr(pos,delimPos-pos);
                        if (token.length()) tokens.push_back(token);
                    }
                }
                pos = delimPos+1;
            }
            else
            {
                if(std::string::npos != tokenPos)
                {
                    std::string token = str.substr(pos);
                    if (token.length()) tokens.push_back(token);
                }
                break;
            }
        }
        return tokens;
    }
};

StringUtils* StringUtils::instance()
{
    static StringUtils s_StringUtils;
    return &s_StringUtils;
}

class AnimController
{
public:
    typedef std::vector<std::string> AnimationMapVector;

    AnimController():
        _model(0),
        _focus(0) {}

    bool setModel(osgAnimation::BasicAnimationManager* model)
    {
        _model = model;
        _map.clear();
        _amv.clear();

        for (osgAnimation::AnimationList::const_iterator it = _model->getAnimationList().begin(); it != _model->getAnimationList().end(); it++)
            _map[(*it)->getName()] = *it;

        for(osgAnimation::AnimationMap::iterator it = _map.begin(); it != _map.end(); it++)
            _amv.push_back(it->first);

        return true;
    }

    bool list(std::ostream &output)
    {
        for(osgAnimation::AnimationMap::iterator it = _map.begin(); it != _map.end(); it++)
            output << "Animation=\"" << it->first << "\"" << std::endl;
        return true;
    }

    bool play()
    {
        if(_focus < _amv.size())
        {
#ifdef _DEBUG
            std::cout << "Play " << _amv[_focus] << std::endl;
#endif
            _model->playAnimation(_map[_amv[_focus]].get());
            return true;
        }

        return false;
    }

    bool play(const std::string& name)
    {
        for(unsigned int i = 0; i < _amv.size(); i++) if(_amv[i] == name) _focus = i;
        _model->playAnimation(_map[name].get());
        return true;
    }

    bool position(const std::string& name, double percentage)
    {
        osgAnimation::Animation* animation = _map[name].get();
        if (animation != 0)
        {
            _model->stopAnimation(animation);

            double startTime = animation->getStartTime();
            double duration = animation->getDuration();
            double time = duration * percentage;

            animation->setWeight(1.f);
            animation->update(startTime+time);
        }
        return true;
    }

    bool stop()
    {
        if(_focus < _amv.size())
        {
#ifdef _DEBUG
            std::cout << "Stop " << _amv[_focus] << std::endl;
#endif
            _model->stopAnimation(_map[_amv[_focus]].get());
            return true;
        }
        return false;
    }

    bool stop(const std::string& name)
    {
        for(unsigned int i = 0; i < _amv.size(); i++) if(_amv[i] == name) _focus = i;
        _model->stopAnimation(_map[name].get());
        return true;
    }

    bool next()
    {
        _focus = (_focus + 1) % _map.size();
#ifdef _DEBUG
        std::cout << "Current now is " << _amv[_focus] << std::endl;
#endif
        return true;
    }

    bool previous()
    {
        _focus = (_map.size() + _focus - 1) % _map.size();
#ifdef _DEBUG
        std::cout << "Current now is " << _amv[_focus] << std::endl;
#endif
        return true;
    }

    const std::string& getCurrentAnimationName() const
    {
        return _amv[_focus];
    }

    const AnimationMapVector& getAnimationMap() const
    {
        return _amv;
    }

private:
    osg::ref_ptr<osgAnimation::BasicAnimationManager> _model;
    osgAnimation::AnimationMap _map;
    AnimationMapVector _amv;
    unsigned int _focus;

};


struct AnimationManagerFinder : public osg::NodeVisitor
{
    osg::ref_ptr<osgAnimation::BasicAnimationManager> _am;
    AnimationManagerFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Node& node) {
#if 0
        if (_am.valid())
            return;osg_filenamef
#endif
        if (node.getUpdateCallback()) {
            osgAnimation::AnimationManagerBase* b = dynamic_cast<osgAnimation::AnimationManagerBase*>(node.getUpdateCallback());
            if (b) {
                _am = new osgAnimation::BasicAnimationManager(*b);
                return;
            }
        }
        traverse(node);
    }
};


class StimulusOSG2: public StimulusInterface
{
public:

StimulusOSG2()
    : _bg_r(0.5)
    , _bg_g(0.5)
    , _bg_b(0.5)
    , _bg_a(1.0)
    , _fade_time(5.0)
{
    ;
}

std::string name() const {
    return "StimulusOSG2";
}

// Derive a class from NodeVisitor to find all MatrixNodes
class MatrixNodeFinder : public osg::NodeVisitor
{
public:
    MatrixNodeFinder( void )
      : osg::NodeVisitor( // Traverse all children.
                osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
        {}

    virtual void apply( osg::Node& node )
    {
        if (typeid(node) == typeid(osg::MatrixTransform))
        {
            _nodeList.push_back(&node);
        }
        // Keep traversing the rest of the scene graph.
        traverse( node );
    }

    osg::NodeList* getNodeList() { return &_nodeList; }

protected:
    osg::NodeList _nodeList;
};

struct MaterialFinderVisitor : public osg::NodeVisitor
{
    MaterialFinderVisitor(osg::Node& node)
        : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
        // We have to be carefull here a bit. Let assume the model
        // has materials (or not). But if it doesn't we want to
        // assign one root material and animate on it. So let check
        // if for a material that might come as a root material for the
        // the model. If there is none, we assign one there
        osg::ref_ptr<osg::StateAttribute> attr = node.getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
        if (!attr.valid())
        {
            // No root material. Assign one
            osg::Material* material = new osg::Material;
            material->setName("@RootMaterial@");
            material->setDataVariance(osg::Object::DYNAMIC);

            node.getOrCreateStateSet()->setAttributeAndModes(material, osg::StateAttribute::ON);

            Alphas alphas;
            getAlphas(material, alphas);

            materials.push_back(std::make_pair(material,alphas));
        }
    }

    virtual void apply( osg::Node& node )
    {
        osg::StateAttribute* attr = node.getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
        if (attr)
        {
#ifdef _DEBUG
            std::cout << "ATTRIBUTE: " << attr->getName() << std::endl;
#endif

            osg::Material* material = dynamic_cast<osg::Material*>(attr);
            if (material && (material->getName() != "@RootMaterial@"))
            {
                material->setDataVariance(osg::Object::DYNAMIC);

                // Get all the original alphas here
                Alphas alphas;
                getAlphas(material,alphas);
#ifdef _DEBUG
                std::cout << "MATERIAL: " << material->getName() << std::endl;
#endif
                materials.push_back(std::make_pair(material,alphas));
            }
        }

        osg::Geode* geode = node.asGeode();
        if (geode)
        {
            for (size_t i=0; i<geode->getNumDrawables(); ++i)
            {
                osg::Drawable* drawable = geode->getDrawable(i);
                osg::StateAttribute* attr = drawable->getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
                if (attr)
                {
#ifdef _DEBUG
                    std::cout << "ATTRIBUTE: " << attr->getName() << std::endl;
#endif

                    osg::Material* material = dynamic_cast<osg::Material*>(attr);
                    if (material && (material->getName() != "@RootMaterial@"))
                    {
                        material->setDataVariance(osg::Object::DYNAMIC);

                        // Get all the original alphas here
                        Alphas alphas;
                        getAlphas(material,alphas);
#ifdef _DEBUG
                        std::cout << "MATERIAL: " << material->getName() << std::endl;
#endif
                        materials.push_back(std::make_pair(material,alphas));
                    }
                }
            }
        }

        traverse(node);
    }

    // postvisit. if no materials were found, override the root one
    void postVisit(osg::Node& node)
    {
        if (materials.size() == 1)
        {
            MaterialAlphaPair& pair = materials.at(0);
            MaterialPtr& material = pair.first;
            node.getOrCreateStateSet()->setAttributeAndModes(material, osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE);
        }
    }

    // This can be tricky here. For simple models, we can change the alpha only of
    // the colors components from the Material. But, if the model already has
    // Materials with alpha set to something else then 1.0, we have to fadeIn to
    // that specific (the original) value. So we have to keep track of the original
    // alphas too, and for each component like ambient, diffuse, emissive and specular
    // and per face, Front and Back

    // Here is proposed structure for alphas
    struct Alphas
    {
        osg::Vec4d  front; // x,y,z,w goes to ambient, diffuse, specular, emission
        osg::Vec4d  back; // x,y,z,w goes to ambient, diffuse, specular, emission
    };

    typedef osg::ref_ptr<osg::Material>         MaterialPtr;
    typedef std::pair<MaterialPtr,Alphas>       MaterialAlphaPair;
    typedef std::vector<MaterialAlphaPair>      Materials;

    Materials   materials;

protected:

    void getAlphas( osg::Material* material, Alphas& alphas)
    {
        alphas.front.x() = material->getDiffuse(osg::Material::FRONT).a();
        alphas.front.y() = material->getAmbient(osg::Material::FRONT).a();
        alphas.front.z() = material->getSpecular(osg::Material::FRONT).a();
        alphas.front.w() = material->getEmission(osg::Material::FRONT).a();
        alphas.back.x() = material->getDiffuse(osg::Material::BACK).a();
        alphas.back.y() = material->getAmbient(osg::Material::BACK).a();
        alphas.back.z() = material->getSpecular(osg::Material::BACK).a();
        alphas.back.w() = material->getEmission(osg::Material::BACK).a();
    }
};

struct FadeInUpdateCallback : public osg::NodeCallback
{
    FadeInUpdateCallback(MaterialFinderVisitor::Materials& materials, double fadeInTimeInSeconds = 10.0)
        : _materials(materials)
        , _fadeInTimeInSeconds(fadeInTimeInSeconds)
        , _fadeInStartTime(0.0)
    {

    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        // Figure out how much we are in the fading, I mean the time
        osg::Timer_t now = osg::Timer::instance()->tick();
        if (_fadeInStartTime == 0.0) {
            _fadeInStartTime = now;
        }

        // This is the time in seconds since the start of the fade in
        double dt = osg::Timer::instance()->delta_s(_fadeInStartTime,now);

        // This is the computed alpha factor
        double factor = osg::minimum( dt / _fadeInTimeInSeconds, 1.0);

#ifdef _DEBUG
        std::cout << "fade factor" << factor << std::endl;
#endif

        // And go through the materials accumulated from the model
        // and apply the alpha factor as well
        MaterialFinderVisitor::Materials::iterator itr = _materials.begin();
        for ( ; itr != _materials.end(); ++itr )
        {
            MaterialFinderVisitor::MaterialAlphaPair& pair = *itr;
            MaterialFinderVisitor::MaterialPtr& material = pair.first;
            MaterialFinderVisitor::Alphas& alphas = pair.second;

            osg::Vec4d color = material->getAmbient(osg::Material::FRONT);
            color.a() = factor*alphas.front.x();
            material->setAmbient(osg::Material::FRONT,color);

            color = material->getDiffuse(osg::Material::FRONT);
            color.a() = factor*alphas.front.y();
            material->setDiffuse(osg::Material::FRONT,color);

            color = material->getSpecular(osg::Material::FRONT);
            color.a() = factor*alphas.front.z();
            material->setSpecular(osg::Material::FRONT,color);

            color = material->getEmission(osg::Material::FRONT);
            color.a() = factor*alphas.front.w();
            material->setEmission(osg::Material::FRONT,color);

            color = material->getAmbient(osg::Material::BACK);
            color.a() = factor*alphas.back.x();
            material->setAmbient(osg::Material::BACK,color);

            color = material->getDiffuse(osg::Material::BACK);
            color.a() = factor*alphas.back.y();;
            material->setDiffuse(osg::Material::BACK,color);

            color = material->getSpecular(osg::Material::BACK);
            color.a() = factor*alphas.back.z();;
            material->setSpecular(osg::Material::BACK,color);

            color = material->getEmission(osg::Material::BACK);
            color.a() = factor*alphas.back.w();;
            material->setEmission(osg::Material::BACK,color);
        }

        // uninstall the callback when w e are done
        if (factor == 1.0) {
            node->setUpdateCallback(0);
        }

    }

protected:
    MaterialFinderVisitor::Materials    _materials;
    double                              _fadeInTimeInSeconds;
    osg::Timer_t                        _fadeInStartTime;
};


void _load_stimulus_filename( std::string osg_filename ) {

    if (!top) {
        std::cerr << "top node not defined!?" << std::endl;
        return;
    }

    ModelDefinition& def = _models[osg_filename];

    // empty submodel map
    def.SubmodelMap.clear();

    // don't show the old switching node.
    top->removeChild(def.switch_node);

    // (create a new switching node.
    def.switch_node = new osg::PositionAttitudeTransform;
    _update_pat(osg_filename);
    top->addChild(def.switch_node);

    // now load it with new contents
    osg::Node *tmp = load_osg_file(osg_filename);  // throws a runtime exception when file not found (catch in display_server.pyx)

    // We are adding extra MatrixTransform node
    // as a root to the loaded model so we attach
    // it to the scene a bit further down
#if 0
    switch_node->addChild(def.tmp);
#endif

    std::cerr << "Loaded " << osg_filename << "\n";

    // Find all "MatrixNode"s
    MatrixNodeFinder fmn;
    tmp->accept( fmn );
    osg::NodeList* nl = fmn.getNodeList();

    std::cerr << "Found " << nl->size() << " osg/MatrixTransform node(s): ";
    for (osg::NodeList::iterator it = nl->begin() ; it != nl->end(); ++it)
    {
        std::cerr << "\n >" << it->get()->getName() << "< ";
        def.SubmodelMap[it->get()->getName()] = it->get();
    }

    // Ok. The deal is, instead of messing around
    // with Matrices from the loaded model - which
    // was the case before and introduced bugs - we
    // now create a root MatrixTransoform that on
    // jason message will move around the model in the
    // scene "bug free". This is our root
    osg::MatrixTransform* root = new osg::MatrixTransform;
    root->addChild(tmp);
    def.SubmodelMap["Root"] = root;

    // Here we do fadein animation
    MaterialFinderVisitor nv(*tmp);
    tmp->accept(nv);
    nv.postVisit(*tmp);
    def.materials = nv.materials;

    // we need to push the node into the transparent bin and enable blending
    // for proper fade in
    tmp->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
    tmp->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);


    // Here we add our new root to the scene
    def.switch_node->addChild(root);

    std::cerr << std::endl;

    // Find all animations
    AnimationManagerFinder finder;
    tmp->accept(finder);
    if (finder._am.valid()) {
        tmp->setUpdateCallback(finder._am.get());
        def._anim.setModel(finder._am.get());
        def._anim.list(std::cout);
    } else {
        osg::notify(osg::WARN) << "no osgAnimation::AnimationManagerBase found in the subgraph, no animations available" << std::endl;
    }
}

void _load_skybox_basename( std::string basename ) {

    if (!top) {
        std::cerr << "top node MaterialAlphaPairnot defined!?" << std::endl;
        return;
    }

    if (skybox_node.valid()) {
        top->removeChild(skybox_node);
        skybox_node = NULL; // dereference the previous node
    }

    if (basename=="<none>") {
        return;
    }

    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;

    if (basename=="<default>") {
        add_default_skybox( mt );
    } else {
        std::string extension = ".png";
        try {
            add_skybox(mt, basename, extension);
        } catch (std::runtime_error) {
            extension = ".jpg";
            add_skybox(mt, basename, extension);
        }

    }

    skybox_node = mt;
    top->addChild(skybox_node);
}

void _update_pat(const std::string& filename)
{
    ModelDefinition& def = _models[filename];

    freemovr_assert(def.switch_node.valid());
    def.switch_node->setPosition( def.model_position );
    def.switch_node->setScale( def.model_scale );

    osg::Matrix m;
    m.makeRotate( def.model_attitude );
    top->setMatrix(m);
}

virtual void post_init(bool slave) {
  top = new osg::MatrixTransform;
  top->addDescription("virtual world top node");

  // when we first start, don't load any model, but create a node that is later deleted.
  // NOTE: Not really sure it is needed since now we are managing
  // multiple models
#if 0
  switch_node = new osg::PositionAttitudeTransform;
  _update_pat();
  top->addChild(switch_node);
#endif

  _virtual_world = top;
}

osg::Vec4 get_clear_color() const {
    return osg::Vec4(_bg_r, _bg_g, _bg_b, _bg_a);
}

void resized(int width,int height) {
}

osg::ref_ptr<osg::Group> get_3d_world() {
    return _virtual_world;
}

osg::ref_ptr<osg::Group> get_2d_hud() {
    return 0;                // use w as scale factor

}

std::vector<std::string> get_topic_names() const {
    std::vector<std::string> result;
    result.push_back("osg2_filename");
    result.push_back("osg2_animation_start");
//    result.push_back("osg2_animation_position");
    result.push_back("osg2_animation_stop");
    result.push_back("osg2_animation_duration");
    result.push_back("osg2_skybox_basename");
    result.push_back("osg2_submodel_pose");
    result.push_back("osg2_submodel_fade_in");
    result.push_back("osg2_background_color");
    return result;
}

void receive_json_message(const std::string& topic_name, const std::string& json_message) {
    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);
    freemovr_assert(root != NULL);

#ifdef _DEBUG
    std::cout << "topic name: " << topic_name << ", json: " << json_message << std::endl;
#endif

    if (topic_name=="osg2_filename")
    {
        std::string fileName = parse_string(root);

        // Special case of unloading all the models by loading /dev/null
        if ( fileName == "/dev/null" )
        {
            ModelDefinitions::iterator itr = _models.begin();
            for ( ; itr != _models.end(); ++itr)
            {
                top->removeChild(itr->second.switch_node);
            }
            _models.clear();
            json_decref(root);
            return;
        }

        // We here agreed if we want to load model with .unload suffix
        // in the file name we simply unload it here, so check for this case first
        std::string::size_type pos = fileName.find(".unload");
        if (pos != std::string::npos)
        {
            fileName = fileName.substr(0,pos);
#ifdef _DEBUG
            std::cout << "Unloading: " << fileName << std::endl;
#endif

            ModelDefinitions::iterator itr = _models.find(fileName);
            if ( itr != _models.end() )
            {
                top->removeChild(itr->second.switch_node);
                _models.erase(itr);
            }
            json_decref(root);
            return;
        }

        // No unload, so load
        // NOTE: This call is creating valid entry in the
        // _models map, which later are refered by the fileName
        _load_stimulus_filename( fileName );

//    } else if (topic_name=="osg2_animation_position") {
//        std::string fileName;
//        std::string animationName;
//        double      animationPercentage = 0.0;
//
//        StringUtils::Tokens tokens = StringUtils::instance()->tokenize(parse_string(root),"|");
//        if (tokens.size() == 2)
//        {
//            fileName = tokens.at(0);
//            animationName = tokens.at(1);
//        }
//        else
//        {
//            std::cerr << "Invalid semantics in \"osg_animation_stop\" message: Expected: file_name|animation_name" << std::endl;
//            return;
//        }
//
//        json_t *nextroot = json_object_get(root, "percentage");
//        animationPercentage = json_number_value(nextroot);
//
//        ModelDefinition& def = _models[fileName];
//        def._anim.position( animationName, animationPercentage );
//
    } else if (topic_name=="osg2_animation_start") {
        StringUtils::Tokens tokens = StringUtils::instance()->tokenize(parse_string(root),"|");
        if (tokens.size() == 2)
        {
            std::string fileName = tokens.at(0);
            std::string animationName = tokens.at(1);

            ModelDefinition& def = _models[fileName];
            def._anim.play( animationName );
        }
        else
        {
            std::cerr << "Invalid semantics in \"osg_animation_start\" message: Expected: file_name|animation_name" << std::endl;
        }
    } else if (topic_name=="osg2_animation_stop") {
        StringUtils::Tokens tokens = StringUtils::instance()->tokenize(parse_string(root),"|");
        if (tokens.size() == 2)
        {
            std::string fileName = tokens.at(0);
            std::string animationName = tokens.at(1);

            ModelDefinition& def = _models[fileName];
            def._anim.stop( animationName );
        }
        else
        {
            std::cerr << "Invalid semantics in \"osg_animation_stop\" message: Expected: file_name|animation_name" << std::endl;
        }
    } else if (topic_name=="osg2_animation_duration") {
        _fade_time = parse_float(root);
    } else if (topic_name=="osg2_background_color") {
        json_t *data_json;
        data_json = json_object_get(root, "r");
        _bg_r = json_number_value( data_json );
        data_json = json_object_get(root, "g");
        _bg_g = json_number_value( data_json );
        data_json = json_object_get(root, "b");
        _bg_b = json_number_value( data_json );
        data_json = json_object_get(root, "a");
        _bg_a = json_number_value( data_json );
    } else if (topic_name=="osg2_skybox_basename") {
        _load_skybox_basename( parse_string(root) );
    } else if (topic_name=="osg2_submodel_fade_in") {
        StringUtils::Tokens tokens = StringUtils::instance()->tokenize(parse_string(root),"|");
        if (tokens.size() == 1)
        {
            std::string fileName = tokens.at(0);
            ModelDefinition& def = _models[fileName];

            // start fade in
            // NOTE: the fadeIn will start when the update callback
            // is installed. The callback uninstall itselfs when the
            // fading is done

            // Sanity check
            if (def.SubmodelMap["Root"] == 0)
            {
                std::cerr << "Message: \"osg_submodel_fade_in\": No root node for model: " << fileName << std::endl;
                json_decref(root);
                return;
            }
            // update callbacks only get run on shown nodes, so we need to make sure its visible
            // which could theoretically cause a one flame flash
            def.SubmodelMap["Root"]->setNodeMask(0xFFFFFFFF);
            def.SubmodelMap["Root"]->setUpdateCallback(new FadeInUpdateCallback(def.materials, _fade_time));
        }
        else
        {
            std::cerr << "Invalid semantics in \"osg_submodel_fade_in\" message: Expected: file_name" << std::endl;
        }
    } else if (topic_name=="osg2_submodel_pose") {
        json_t *data_json;
        json_t *nextroot;
        nextroot = json_object_get(root, "header");
        data_json = json_object_get(nextroot, "frame_id");
        std::string name = json_string_value(data_json);
        nextroot = json_object_get(root, "pose");
        data_json = json_object_get(nextroot, "position");
        osg::Vec3 position = parse_vec3(data_json);
        data_json = json_object_get(nextroot, "orientation"); // we interpret the quaternion as 3 Euler angles and w as scalefactor (HACK!)
        osg::Quat attitude = parse_quat(data_json);
#ifdef _DEBUG
        std::cout << "osg2_submodel_pose: name: " << name <<
        " pos: " << position.x() << ", " << position.y() << ", " << position.z() <<
        " euler orientation: " << attitude.x()  << ", " << attitude.y()  << ", " << attitude.z()  <<
        " scale: " << attitude.w()  << std::endl;
#endif
        try {
            ModelDefinition& def = _models[name];

            // We have agreed that the animated model will be
            // one per file, so when we want to move the animated
            // model we have changed it now to move the root of
            // the whole model - NOTE: we are adding the root on
            // the file load
            name = "Root";
            osg::Node* n = def.SubmodelMap.at(name);

            // Just a bit of paranoia (otherwise will crash)
            if (n == 0) throw std::out_of_range("No root node for this file");

            osg::Matrix m;
            m.makeIdentity();

            // use xyz as Euler angles
            osg::Quat q=osg::Quat(
                attitude.x(), osg::Vec3(1.0,0.0,0.0),   // rotate around X-axis
                attitude.y(), osg::Vec3(0.0,1.0,0.0),   // rotate around Y-axis
                attitude.z(), osg::Vec3(0.0,0.0,1.0));  // rotate around Z-axis

            osg::Matrixd rotateMx;
            rotateMx.makeRotate(q);

            osg::Matrixd translateMx;
            translateMx.makeTranslate(position);

            osg::Matrixd scaleMx;
            if (attitude.w() > 0) {
                // use w as scale factor
                scaleMx.makeScale(osg::Vec3(attitude.w(), attitude.w(), attitude.w()));
                // show
                n->setNodeMask(0xFFFFFFFF);
            } else if (attitude.w() == 0.0) {
                // hide
                n->setNodeMask(0x0);
            }
            m = scaleMx * rotateMx * translateMx;

            ((osg::MatrixTransform *)n)->setMatrix(m);

        } catch (std::out_of_range) {
            std::cerr << " node '" << name << "' does not exist!\n";
        }
    } else {
        throw std::runtime_error("unknown topic name");
    }

    json_decref(root);
}

std::string get_message_type(const std::string& topic_name) const {
    std::string result;

    if (topic_name=="osg2_filename") {
        result = "std_msgs/String";
    } else if (topic_name=="osg2_animation_start") {
        result = "std_msgs/String";
    } else if (topic_name=="osg2_animation_stop") {
        result = "std_msgs/String";
//    } else if (topic_name=="osg2_animation_position") {
//        result = "std_msgs/String";
    } else if (topic_name=="osg2_animation_duration") {
        result = "std_msgs/Float32";
    } else if (topic_name=="osg2_skybox_basename") {
        result = "std_msgs/String";
    } else if (topic_name=="osg2_submodel_pose") {
        result = "geometry_msgs/PoseStamped";
    } else if (topic_name=="osg2_submodel_fade_in") {
        result = "std_msgs/String";
    } else if (topic_name=="osg2_background_color") {
        result = "std_msgs/ColorRGBA";
    } else {
        throw std::runtime_error(format("unknown topic name: %s",topic_name));
    }
    return result;

}

private:
    float _bg_r, _bg_g, _bg_b, _bg_a;
    float _fade_time;

    osg::ref_ptr<osg::Group>            _virtual_world;
    osg::ref_ptr<osg::MatrixTransform>  skybox_node;
    osg::ref_ptr<osg::MatrixTransform>  top;

    // We have changed now to support multiple
    // models management so this is my first guess
    // from the code read so far what is model
    // specific. The key for each model is the filename
    // since it is guaranteed the file names will be unique
    struct ModelDefinition
    {
        osg::ref_ptr<osg::PositionAttitudeTransform> switch_node;

        osg::Vec3 model_scale;
        osg::Vec3 model_position;

        // NOTE: not sure about the following
        // one if this has to be per model since it is set
        // for the top Matrix, and the top Matrix is containing
        // all of the models. Please review
        osg::Quat model_attitude;

        std::map<std::string, osg::ref_ptr<osg::Node> > SubmodelMap;
        AnimController _anim;

        MaterialFinderVisitor::Materials materials;

        ModelDefinition()
            : model_scale(1,1,1)
        {

        }
    };

    typedef std::map< std::string, ModelDefinition >        ModelDefinitions;
    ModelDefinitions    _models;
};


POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusOSG2)
POCO_END_MANIFEST

void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}


