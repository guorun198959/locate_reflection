#include <iostream>
#include <chrono>
#include <boost/bind.hpp>
// node msg and srv
#include <catkin_startup/mytopic.h>
#include <catkin_startup/myservice.h>
#include <cpp_utils/container.h>
#include <cpp_utils/listener.h>
#include <cpp_utils/threading.h>
#include <cpp_utils/parse.h>

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
#include <tf/tf.h>


//
#include <XmlRpc.h>

#include <yaml/Yaml.hpp>
#include <vector>
#include <string>
#include <map>
#include <tuple>
#include <thread>
using std::vector;
using std::string;
using std::map;
using std::tuple;
namespace node=catkin_startup;



// how to create search tree for flann or nanoann
// given latest baselink
// and a set of points[x,y,yaw]


double normalDiff(double angle1, double angle2) {

    double diff = angle1 - angle2;
    diff = atan2(sin(diff), cos(diff));
    return fabs(diff);
}
struct Position {
    double x;
    double y;
    double yaw;
};


class KdTreeCreator {

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::shared_ptr<sensor_msgs::LaserScan> laser_data_;
    geometry_msgs::Pose laserPose_;
    XmlRpc::XmlRpcValue value_;

    Yaml::Node param_;


    void getLaserPose();

    util::Listener l;

    void getBoardPosition();

    vector<Position> xmlToPoints();
public:
    KdTreeCreator(ros::NodeHandle nh, ros::NodeHandle nh_provate);



};


KdTreeCreator::KdTreeCreator(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private),
                                                                               l(nh, nh_private) {
//    util::Listener l(nh, nh_private);
    auto res = l.createSubcriberFilteredTf<sensor_msgs::LaserScan>("scan", 2, "map");
    laser_data_ = std::get<0>(res);
    l.getOneMessage("scan", -1);
    cout << laser_data_.get()->header;
    getLaserPose();
    getBoardPosition();


}

void KdTreeCreator::getLaserPose() {

    ROS_INFO("getLaserPose start tf");
//    l.getOneMessage("scan",-1);
    tf::Transform transform;
    l.getTransform("map", "laser", transform);
    tf::poseTFToMsg(transform, laserPose_);
    ROS_INFO_STREAM(laserPose_);
}

void KdTreeCreator::getBoardPosition() {

    string filename = "/home/waxz/refloc_ws/src/catkin_startup/launch/board.yaml";

    param_ = util::readFile(filename);
    xmlToPoints();

}

vector<Position> KdTreeCreator::xmlToPoints() {
    double visibel_angle = param_["visibel_angle"].As<double>();
    double visibel_range_ratio = param_["visibel_range_ratio"].As<double>();

    vector<Position> positions;

    positions.clear();

    Position p;


    util::createMapFromXmlRpcValue<double>(value_[0]);
//    return positions;

    for (int it = 0; it < param_["board_position"].Size(); it++) {

        p.x = param_["board_position"][it]["x"].As<double>();

        p.y = param_["board_position"][it]["y"].As<double>();

        p.yaw = param_["board_position"][it]["yaw"].As<double>();

        cout << "ffff" << value_[it]["x"];


        // check visibility
        double yaw = tf::getYaw(laserPose_.orientation);
        ROS_INFO("board %.3f,%.3f,%.3f,%.3f,%.3f,%.3f", p.x, p.y, p.yaw, laserPose_.position.x, laserPose_.position.y,
                 yaw);

        double distance = sqrt(pow(laserPose_.position.x - p.x, 2) +
                               pow(laserPose_.position.y - p.y, 2));
        if (distance > visibel_range_ratio * laser_data_.get()->range_max && distance < 0.1)
            continue;

        double boardtorobotangle = atan2(laserPose_.position.y - p.y, laserPose_.position.x - p.x);
        double robottoboardangle = atan2(p.y - laserPose_.position.y, p.x - laserPose_.position.x);

//        bool in_view = robottoboardangle > laser_data_.get()->angle_min && robottoboardangle < laser_data_.get()->angle_max;
        double direction1 = normalDiff(boardtorobotangle, p.yaw);
        double direction2 = normalDiff(robottoboardangle, yaw);
        bool condition2 = direction2 > laser_data_.get()->angle_min && direction2 < laser_data_.get()->angle_max;
        bool condition1 = direction1 < visibel_angle;

        ROS_INFO("data inrange,%.3f,%.3f,%.3f,%.3f", robottoboardangle, boardtorobotangle, direction1, direction2);


        if (condition1 && condition2) {
            positions.push_back(p);
        }

    }
    ROS_INFO("get boarder:%d", int(positions.size()));

    return positions;

}

void createSearchTree(XmlRpc::XmlRpcValue value, geometry_msgs::Pose laserPose) {

//    vector<Position> positions = xmlToPoints(value);


    // fist compute or point's distance to robot fll_set --> set1
    // compute direction, set1 --> set2

}


void Read(string fn) {
    using namespace Yaml;
    Yaml::Node root;
    Yaml::Parse(root, "/home/waxz/refloc_ws/src/catkin_startup/launch/board.yaml");

// Print all scalars.
    std::cout << root["visibel_angle"].As<double>() << std::endl;
    std::cout << root["visibel_range_ratio"].As<double>() << std::endl;
//
    std::cout << "size" << root["board_position"].Size();
    std::cout << root["board_position"][0]["x"].As<double>() << std::endl;
    std::cout << root["board_position"][1]["y"].As<double>() << std::endl;

    Serialize(root, "/home/waxz/refloc_ws/src/catkin_startup/launch//out.txt");

//// Iterate second sequence item.
//    Node & item = root[1];
//    for(auto it = item.Begin(); it != item.End(); it++)
//    {
//        std::cout << (*it).first << ": " << (*it).second.As<string>() << std::endl;
//    }

}



int main(int argc, char **argv) {

    string filename = "/home/waxz/refloc_ws/src/catkin_startup/launch/board.yaml";

    Yaml::Node param = util::readFile(filename);
    cout << param["board_position"][0]["yaw"].As<double>();
    std::cout << param["board_position"][0]["x"].As<double>() << std::endl;




//    return 0;



    std::cout << "Hello, World!" << std::endl;
    //init a node
    ros::init(argc, argv, "start_up");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // test xmlrpcvalue
#if 0
    XmlRpc::XmlRpcValue value;
    nh.getParam("board_position", value);
    auto V = util::createVectorFromXmlRpcValue(value);
    ROS_INFO_STREAM(value[0]["x"]);
    cout << V[0]["x"];
#endif
    ros::Duration(0.5).sleep();
    KdTreeCreator kd(nh, nh_private);


    return 0;

    //**** topic
    // publish on o topic
    ros::Publisher chat_pub = nh.advertise<node::mytopic>("chat",1);

    util::Listener l(nh, nh_private);
    std::shared_ptr<node::mytopic> p1 = l.getChat<node::mytopic>("chat");
    if (p1)
        ROS_INFO("%s",p1.get()->cmd.data.c_str());
    node::mytopic msg;
    msg.header.frame_id = "node";

//    boost::shared_ptr<node::mytopic> data = boost::shared_ptr<node::mytopic>(new node::mytopic());
//    data->cmd.data = "sd";
    ros::spinOnce();
    ros::Rate(1).sleep();

#if 0
    auto res = l.createSubcriber<node::mytopic>("chat", 2);
    std::shared_ptr<node::mytopic> data = std::get<0>(res);
#endif
    //filter
    auto res = l.createSubcriberFilteredTf<node::mytopic>("chat", 2, "map");
    std::shared_ptr<node::mytopic> data = std::get<0>(res);
    auto res2 = l.createSubcriberFilteredTf<sensor_msgs::LaserScan>("scan", 2, "map");
    std::shared_ptr<sensor_msgs::LaserScan> data2 = std::get<0>(res2);

#if 1

    tf::Transform transform;
    l.getTransform("map", "laser", transform);
#endif
    l.getOneMessage("chat", 0.1);
    cout << data2.get()->range_max;
    ROS_INFO_STREAM(data2.get()->header);
#if 0
    util::Threading t;
    util::Task<node::mytopic> task(100);
    util::MyTask<node::mytopic> mytask(100,data);
    util::ThreadPublisher<node::mytopic> pub_task(10,data,nh);
    t.createThread(task);
    t.createThread(mytask);
    t.createThread(pub_task);
    pub_task.start();
#endif

    ros::Rate rate(10);
//    return 0;
    int i=0;
    while (ros::ok()){
        i++;
        chat_pub.publish(msg);


        char tmp[200];
        sprintf(tmp,"start %d",i);
        msg.cmd.data =string(tmp) ;
#if 0
        task.chat(tmp);
        if (i ==5){

//            task.start();
        }
        if (i == 10){
            mytask.start();
            task.exit();

            mytask.chat("666666");
        }

        if (i ==20)
            mytask.exit();


#endif
        ros::spinOnce();

//        l.getOneMessage("chat",0.1);
//        if(data){
//            ROS_INFO("local receive %s",data.get()->cmd.data.c_str());
//        }
#if 1
        l.getOneMessage("scan", 0.5);
        if(data){
            ROS_INFO("local receive %s",data.get()->cmd.data.c_str());
        }
#endif



        rate.sleep();
        ros::spinOnce();


    }

    // test code



    return 0;
}