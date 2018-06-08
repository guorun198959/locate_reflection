//
// Created by waxz on 18-6-8.
//

#ifndef CATKIN_STARTUP_MARCO_UTIL_H
#define CATKIN_STARTUP_MARCO_UTIL_H

#include <iostream>
#define DBGVAR( var ) \
  (std::cout) << "DBG: "<<__TIME__<<"file:" << __FILE__ << "(line:" << __LINE__ <<",function:"<<__FUNCTION__ <<") "\
       << #var << " = [" << (var) << "]" << std::endl


/*usage
std::string tt = "tt";
DBGVAR(tt );
*/
#endif //CATKIN_STARTUP_MARCO_UTIL_H
