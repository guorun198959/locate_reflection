//
// Created by waxz on 18-7-17.
//

#ifndef LOCATE_REFLECTION_STRING_H
#define LOCATE_REFLECTION_STRING_H

#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

namespace string_util {
    using namespace std;

    std::vector<std::string> split(const std::string &s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }
}
#endif //LOCATE_REFLECTION_STRING_H
