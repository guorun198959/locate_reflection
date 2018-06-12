#include <iostream>
#include <chrono>
#include <boost/bind.hpp>
// node msg and srv
#include <catkin_startup/mytopic.h>
#include <catkin_startup/myservice.h>
#include <cpp_utils/container.h>
#include <cpp_utils/listener.h>
#include <cpp_utils/threading.h>

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

#include <XmlRpc.h>

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

    void getLaserPose();

    util::Listener l;
public:
    KdTreeCreator(ros::NodeHandle nh, ros::NodeHandle nh_provate);



};


KdTreeCreator::KdTreeCreator(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private),
                                                                               l(nh, nh_private) {
//    util::Listener l(nh, nh_private);
    auto res = l.createSubcriber<sensor_msgs::LaserScan>("scan", 2);
    laser_data_ = std::get<0>(res);
    l.getOneMessage("scan", -1);
    cout << laser_data_.get()->header;
    getLaserPose();


}

void KdTreeCreator::getLaserPose() {

    ROS_INFO("getLaserPose start tf");
//    l.getOneMessage("scan",-1);
    tf::Transform transform;
    l.getTransform("map", "laser", transform);
    tf::poseTFToMsg(transform, laserPose_);
    ROS_INFO_STREAM(laserPose_);
}


vector<Position> xmlToPoints(XmlRpc::XmlRpcValue value, geometry_msgs::Pose laserPose) {

    vector<Position> positions;

    positions.clear();

    Position p;

    for (size_t it = 0; it < value.size(); it++) {

        p.x = value[it]["x"];

        p.y = value[it]["y"];

        p.yaw = value[it]["yaw"];

        // check visibility
        double yaw = tf::getYaw(laserPose.orientation);
        double distance = sqrt(pow(laserPose.position.x - p.x, 2) +
                               pow(laserPose.position.y - p.y, 2));
        if (distance > 20)
            continue;

        double boardtorobotangle = atan2(laserPose.position.y - p.y, laserPose.position.x - p.x);


        positions.push_back(p);

    }
    return positions;

}

void createSearchTree(XmlRpc::XmlRpcValue value, geometry_msgs::Pose laserPose) {

//    vector<Position> positions = xmlToPoints(value);


    // fist compute or point's distance to robot fll_set --> set1
    // compute direction, set1 --> set2

}

int main(int argc, char **argv) {



    std::cout << "Hello, World!" << std::endl;
    //init a node
    ros::init(argc, argv, "start_up");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // test xmlrpcvalue
    XmlRpc::XmlRpcValue value;
    nh.getParam("board_position", value);
    auto V = util::createVectorFromXmlRpcValue(value);
    ROS_INFO_STREAM(value[0]["x"]);
    cout << V[0]["x"];

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