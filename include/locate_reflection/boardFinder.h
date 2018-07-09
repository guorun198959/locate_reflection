//
// Created by waxz on 18-7-4.
//

#ifndef LOCATE_REFLECTION_BOARDFINDER_H
#define LOCATE_REFLECTION_BOARDFINDER_H

#include <iostream>
#include <chrono>
#include <boost/bind.hpp>
// node msg and srv

#include <cpp_utils/container.h>
#include <cpp_utils/listener.h>
#include <cpp_utils/threading.h>
#include <cpp_utils/parse.h>
#include <cpp_utils/search.h>
#include <locate_reflection/patternMatcher.h>

#include <ros/ros.h>
#include <ros/callback_queue.h> //  ros::CallbackQueue queue
#include "tf/message_filter.h"  // filter message with tf
#include <message_filters/subscriber.h>


//sensor data
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

//geometry transformation
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

#include <Eigen/Dense>


//

#include <yaml/Yaml.hpp>
#include <kdtree/kdtree.h>
#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <thread>
#include <array>

using std::vector;
using std::string;
using std::map;
using std::tuple;
using std::endl;


// normal angle diffrence
inline double normalDiff(double angle1, double angle2) {

    double diff = angle1 - angle2;
    // normal to -pi,pi
    diff = atan2(sin(diff), cos(diff));

    return fabs(diff);
}

inline bool angleCompare(const Position p1, const Position p2) {
    return atan2(p1.y, p1.x) > atan2(p2.y, p2.x);

}


class BoardFinder {

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformBroadcaster *tfb_;
    std::shared_ptr<sensor_msgs::LaserScan> laser_data_;
    geometry_msgs::Pose laserPose_;
    valarray<float> bear_;
    tf::Transform baseLaserTf_;
    tf::Transform mapOdomTf_;
    std::shared_ptr<geometry_msgs::Pose> mapOdom_data_;

    // psrameter
    string scan_topic_;
    string odomtf_topic_;

    // read config file
    Yaml::Node param_;

    // pattern matcher
    PatternMatcher patternMatcher;

    // a thread for publishing tf
    threading_util::ThreadClass threadClass_;
    threading_util::Func_tfb tfThread_;
    std::shared_ptr<tf::StampedTransform> mapToodomtfPtr_;
//    threading_util::Threading<threading_util::ThreadTfPub<tf::StampedTransform>> threading_;
//    threading_util::ThreadTfPub<tf::StampedTransform> threadTfPub_;




    bool getLaserPose();

    rosnode::Listener l;
    ros::Publisher boardPub, pointPub, markerPub;


    bool xmlToPoints(vector<Position> &pointsW);

    bool findNN(vector<Position> &realPointsW, vector<Position> &realPoints, vector<Position> &detectPoints);

    void computeUpdatedPose(vector<Position> realPoints, vector<Position> detectPoints);

    void updateMapOdomTf(tf::Transform laserPose, ros::Time time);

    bool transformPoints(vector<Position> &realPoints);


    bool updateSensor();

    void updateSharedData(tf::Transform mapTOodomTf);

public:

    BoardFinder(ros::NodeHandle nh, ros::NodeHandle nh_private);

    ~BoardFinder();

    bool getMapOdomTf();

    bool getBoardPosition(vector<Position> &pointsW, vector<Position> &points);

    vector<Position> detectBoard();

    void findLocation();


};


#endif //LOCATE_REFLECTION_BOARDFINDER_H
