#include <jansson.h>
#include <osg/Vec3>
#include <osg/Quat>
#include <string>

osg::Vec3 parse_vec3(json_t* root);
osg::Quat parse_quat(json_t* root);
float parse_float(json_t* root);
std::string parse_string(json_t* root);
int parse_int(json_t* root);
