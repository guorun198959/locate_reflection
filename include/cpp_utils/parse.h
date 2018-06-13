//
// Created by waxz on 18-6-12.
//

#ifndef CATKIN_STARTUP_PARSE_H
#define CATKIN_STARTUP_PARSE_H

#include <yaml/Yaml.hpp>
#include <string>

using std::string;
namespace util {

    Yaml::Node readFile(string filename) {
        using namespace Yaml;
        Yaml::Node root;
        Yaml::Parse(root, filename.c_str());
        return root;
    }
}
#endif //CATKIN_STARTUP_PARSE_H
