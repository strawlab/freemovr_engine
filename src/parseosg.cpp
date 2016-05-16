#include <iostream>
#include <osg/io_utils>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osgDB/ReadFile>
#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Bone>

class MatrixNodeFinder : public osg::NodeVisitor
{
public:
    MatrixNodeFinder( void )
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
        {}

    virtual void apply( osg::Node& node )
    {
        if (typeid(node) == typeid(osg::MatrixTransform))
        {
            _nodeList.push_back(&node);
        }
        traverse( node );
    }

    osg::NodeList* getNodeList() { return &_nodeList; }

protected:
    osg::NodeList _nodeList;
};

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
            output << "Animation=" << it->first << std::endl;
        return true;
    }

    bool play() 
    {
        if(_focus < _amv.size()) 
        {
            //std::cout << "Play " << _amv[_focus] << std::endl;
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

    bool stop() 
    {
        if(_focus < _amv.size()) 
        {
            //std::cout << "Stop " << _amv[_focus] << std::endl;
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
        //std::cout << "Current now is " << _amv[_focus] << std::endl;
        return true;
    }

    bool previous() 
    {
        _focus = (_map.size() + _focus - 1) % _map.size();
        //std::cout << "Current now is " << _amv[_focus] << std::endl;
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
        if (_am.valid())
            return;
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

int main(int argc, char** argv) 
{
    osg::ArgumentParser arguments(&argc, argv);
    arguments.getApplicationUsage()->setApplicationName("parseosg");
    arguments.getApplicationUsage()->addCommandLineOption("-h or --help","List command line options.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename.osg");

    if (arguments.read("-h") || arguments.read("--help"))
    {
        arguments.getApplicationUsage()->write(std::cout, osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 0;
    }

    if (arguments.argc()<=1)
    {
        arguments.getApplicationUsage()->write(std::cout, osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 1;
    }

    std::string osgfile = arguments[1];
    osg::Node* node = dynamic_cast<osg::Node*>(osgDB::readNodeFile(osgfile));
    if(!node)
    {
        std::cout << arguments.getApplicationName() <<": No data loaded" << std::endl;
        return 1;
    }

    // list MatrixTranform nodes
    MatrixNodeFinder fmn;
    node->accept( fmn );
    osg::NodeList* nl = fmn.getNodeList();
    for (osg::NodeList::iterator it = nl->begin() ; it != nl->end(); ++it)
        std::cout << "MatrixTransformNode=" << it->get()->getName() << std::endl;

    // list animatable nodes
    AnimController anim;
    AnimationManagerFinder finder;
    node->accept(finder);
    if (finder._am.valid()) {
        node->setUpdateCallback(finder._am.get());
        anim.setModel(finder._am.get());
        anim.list(std::cout);
    }

    return 0;
}
