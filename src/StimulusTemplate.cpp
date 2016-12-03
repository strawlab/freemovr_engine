/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

/* StimulusTemplate

   This is a completely empty, do-nothing stimulus that is useful for
   using as a starting point for writing other plugins.

 */

#include "freemoovr_engine/StimulusInterface.hpp"
#include "Poco/ClassLibrary.h"
#include <stdexcept>

class StimulusTemplate: public StimulusInterface
{
public:
StimulusTemplate() {
}

std::string name() const {
    return "StimulusTemplate";
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
};

POCO_BEGIN_MANIFEST(StimulusInterface)
POCO_EXPORT_CLASS(StimulusTemplate)
POCO_END_MANIFEST

// optional set up and clean up functions
void pocoInitializeLibrary()
{
}

void pocoUninitializeLibrary()
{
}
