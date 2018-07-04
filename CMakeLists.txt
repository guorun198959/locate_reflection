cmake_minimum_required(VERSION 2.8.3)
project(catkin_startup)


# C++ compile flags
# -std=c++11 -> for c++ standarts
# -Wall for WARNING
# -pedantic -> for warning between c and c++
# -pthread for portability
# -g -O0 -fprofile-arcs -ftest-coverage for code coverage
if(NOT CMAKE_VERSION VERSION_LESS 3.1)
    set(CMAKE_CXX_STANDARD 11)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -pthread -g -O0 -fprofile-arcs -ftest-coverage")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -pthread -g -O0 -fprofile-arcs -ftest-coverage")
# Build warning with -pedantic https://github.com/ros/rosconsole/issues/9
# example how to set c++ compiler flags for GNU
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wno-unknown-pragmas -Wno-sign-compare -Woverloaded-virtual -Wwrite-strings -Wno-unused")
    set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif()



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(
        catkin REQUIRED COMPONENTS cmake_modules geometry_msgs sensor_msgs tf nav_msgs
        roscpp amcl
)
find_package(Eigen REQUIRED)


#define srv and msg
add_service_files(FILES
        myservice.srv
        )

add_message_files(
        FILES
        mytopic.msg
)

generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs
        )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
        DEPENDS Eigen
)
include_directories(
        include
        src
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS})


# add configure file
configure_file(
        "${PROJECT_SOURCE_DIR}/launch/board.yaml"
        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/board.yaml"
)
message("== ${PROJECT_SOURCE_DIR} , ${PROJECT_BINARY_DIR},${CMAKE_RUNTIME_OUTPUT_DIRECTORY},${CMAKE_BINARY_DIR}")

add_library(${PROJECT_NAME} src/reflection_localization.cpp)
#
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
#




install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)


#### test node

add_executable(${PROJECT_NAME}_node src/main.cpp include/yaml/Yaml.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(matrix src/start_up.cpp include/yaml/Yaml.cpp)
target_link_libraries(matrix)


install(
        TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

