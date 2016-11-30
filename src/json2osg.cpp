#include "json2osg.hpp"
#include "freemoovr/freemoovr_assert.h"

#include "base64.h"
#include <stdexcept>
#include <jansson.h>

osg::Vec3 parse_vec3(json_t* root) {
    json_t *data_json;
    double x,y,z;

    data_json = json_object_get(root, "x");
    freemoovr_assert(data_json != NULL);
    freemoovr_assert(json_is_number(data_json));
    x = json_number_value( data_json );

    data_json = json_object_get(root, "y");
    freemoovr_assert(data_json != NULL);
    freemoovr_assert(json_is_number(data_json));
    y = json_number_value( data_json );

    data_json = json_object_get(root, "z");
    freemoovr_assert(data_json != NULL);
    freemoovr_assert(json_is_number(data_json));
    z = json_number_value( data_json );

    return osg::Vec3(x,y,z);
}

osg::Quat parse_quat(json_t* root) {
    json_t *data_json;
    double x,y,z,w;

    data_json = json_object_get(root, "x");
    freemoovr_assert(data_json != NULL);
    freemoovr_assert(json_is_number(data_json));
    x = json_number_value( data_json );

    data_json = json_object_get(root, "y");
    freemoovr_assert(data_json != NULL);
    freemoovr_assert(json_is_number(data_json));
    y = json_number_value( data_json );

    data_json = json_object_get(root, "z");
    freemoovr_assert(data_json != NULL);
    freemoovr_assert(json_is_number(data_json));
    z = json_number_value( data_json );

    data_json = json_object_get(root, "w");
    freemoovr_assert(data_json != NULL);
    freemoovr_assert(json_is_number(data_json));
    w = json_number_value( data_json );

    return osg::Quat(x,y,z,w);
}

bool parse_bool(json_t* root) {
    json_t *data_json;

    data_json = json_object_get(root, "data");
    freemoovr_assert(json_is_boolean(data_json));

    return json_is_true(data_json);
}

int parse_int(json_t* root) {
    json_t *data_json;

    data_json = json_object_get(root, "data");
    freemoovr_assert(json_is_integer(data_json));

    return json_integer_value(data_json);
}

float parse_float(json_t* root) {
    json_t *data_json;

    data_json = json_object_get(root, "data");
    freemoovr_assert(json_is_real(data_json));

    return json_real_value(data_json);
}

std::string parse_string(json_t* root) {
    json_t *data_json;

    data_json = json_object_get(root, "data");
    freemoovr_assert(json_is_string(data_json));

    return json_string_value(data_json);
}

std::vector<double> parse_vector_double(json_t* root) {
   json_t *data_json;
   std::vector<double> result;

   freemoovr_assert(json_is_array(root));

   for (size_t i=0; i<json_array_size(root); i++) {
     data_json = json_array_get( root, i );
     freemoovr_assert(json_is_real(data_json));
     double val = json_real_value(data_json);
     result.push_back(val);
   }

   return result;
}

void parse_json_image(const std::string& json_message,
                      std::string& image_format,
                      std::string& image_data) {

    json_t *root;
    json_error_t error;

    root = json_loads(json_message.c_str(), 0, &error);

    if(!root) {
		fprintf(stderr, "error: in %s(%d) on json line %d: %s\n", __FILE__, __LINE__, error.line, error.text);
		throw std::runtime_error("error in json file");
    }

    json_t *image_data_base64_json = json_object_get(root, "data (base64)");
    if(!json_is_string(image_data_base64_json)){
		fprintf(stderr, "error: in %s(%d): expected string\n", __FILE__, __LINE__);
		throw std::runtime_error("error in json file");
    }

    json_t *image_format_json = json_object_get(root, "format");
    if(!json_is_string(image_format_json)){
		fprintf(stderr, "error: in %s(%d): expected string\n", __FILE__, __LINE__);
		throw std::runtime_error("error in json file");
    }

    std::string image_data_base64( json_string_value( image_data_base64_json ) );
    image_format = std::string( json_string_value( image_format_json ));

    image_data = std::string( base64_decode( image_data_base64 ));
    json_decref(root);
}
