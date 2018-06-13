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
#include <geometry_msgs/PoseArray.h>
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

ros::Publisher boardPub, pointPub;

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
    std::shared_ptr<ros::Subscriber> sub_;
    std::shared_ptr<sensor_msgs::LaserScan> laser_data_;
    geometry_msgs::Pose laserPose_;
    XmlRpc::XmlRpcValue value_;
    valarray<float> bear_;

    Yaml::Node param_;


    void getLaserPose();

    util::Listener l;


    void getBoardPosition();

    vector<Position> xmlToPoints();
public:
    void detectBoard();

    bool updateSensor();
    KdTreeCreator(ros::NodeHandle nh, ros::NodeHandle nh_provate);



};


KdTreeCreator::KdTreeCreator(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private),
                                                                               l(nh, nh_private) {
//    util::Listener l(nh, nh_private);
#if 0
    auto res = l.createSubcriberFilteredTf<sensor_msgs::LaserScan>("scan", 2, "map");

#endif
#if 1

    auto res = l.createSubcriber<sensor_msgs::LaserScan>("scan", 10);
#endif


    laser_data_ = std::get<0>(res);
//    sub_ = std::get<1>(res);
    l.getOneMessage("scan", -1);
    cout << "233" << laser_data_.get()->header;
//    updateSensor();



//    getLaserPose();
    getBoardPosition();


}

bool KdTreeCreator::updateSensor() {
    bool getmsg = l.getOneMessage("scan", -1);
    cout << "255" << laser_data_.get()->header;
    size_t size = laser_data_.get()->ranges.size();
    if (bear_.size() != size) {
        bear_ = valarray<float>(0.0, size);
        for (int i = 0; i < size; i++) {
            bear_[i] = laser_data_.get()->angle_min + i * laser_data_.get()->angle_increment;
        }
    }
    return getmsg;
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
//    xmlToPoints();

}

vector<Position> KdTreeCreator::xmlToPoints() {
    auto visibel_angle = param_["visibel_angle"].As<double>();
    auto visibel_range_ratio = param_["visibel_range_ratio"].As<double>();

    vector<Position> positions;

    positions.clear();

    Position p;


    util::createMapFromXmlRpcValue<double>(value_[0]);
//    return positions;




    for (int it = 0; it < param_["board_position"].Size(); it++) {

        p.x = param_["board_position"][it]["x"].As<double>();

        p.y = param_["board_position"][it]["y"].As<double>();

        p.yaw = param_["board_position"][it]["yaw"].As<double>();



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


void KdTreeCreator::detectBoard() {
    // upddate sensor
    ROS_INFO("start scan");
    if (!updateSensor())
        return;;

    // vector to valarray
    valarray<float> ranges = util::createValarrayFromVector<float>(laser_data_.get()->ranges);
    valarray<float> intensities = util::createValarrayFromVector<float>(laser_data_.get()->intensities);
    if (intensities.size() == 0) {
        ROS_ERROR("scan has no intensities ");
        exit(0);
    }

    valarray<float> xs = ranges * cos(bear_);
    valarray<float> ys = ranges * sin(bear_);


    auto intensity_thresh = param_["intensity_thresh"].As<float>();
    auto neighbor_thresh = param_["neighbor_thresh"].As<float>();
    auto length_thresh = param_["length_thresh"].As<float>();


//    valarray<float> lightPoitns = intensities[intensities>intensity_thresh];
    valarray<float> lightXs = xs[intensities > intensity_thresh];
    valarray<float> lightYs = ys[intensities > intensity_thresh];

    size_t size = lightXs.size();
    if (size == 0) {

        ROS_ERROR("no light point");
        return;

    }

    valarray<float> lightXsL = lightXs[std::slice(0, size - 1, 1)];
    valarray<float> lightXsR = lightXs[std::slice(1, size - 1, 1)];

    valarray<float> lightYsL = lightYs[std::slice(0, size - 1, 1)];
    valarray<float> lightYsR = lightYs[std::slice(1, size - 1, 1)];

    valarray<float> distance = sqrt(pow(lightXsR - lightXsL, 2) + pow(lightYsR - lightYsL, 2));


//    for (int i =0;i< distance.size();i++)
//        cout<<distance[i]<<",";


    Position p;
    vector<Position> ps;
    size_t pointNum = 0;
    double pointLength = 0;

    //publish pose
    geometry_msgs::PoseArray msg, lightpoints;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser";
    lightpoints.header.stamp = ros::Time::now();
    lightpoints.header.frame_id = "laser";
    geometry_msgs::Pose pose, lightpose;


    for (int i = 0; i < distance.size(); i++) {

        if (distance[i] < neighbor_thresh && i < distance.size() - 2) {
            pointNum++;
        }
        if (distance[i] > neighbor_thresh || (i == distance.size() - 1 && distance[i] < neighbor_thresh)) {
            double length = sqrt(
                    pow(lightXsL[i] - lightXsL[i - pointNum], 2) + pow(lightYsL[i] - lightYsL[i - pointNum], 2));
            //compute length

            if (length > length_thresh) {

                // get index
                vector<size_t> idx_vec;
                for (size_t it = i - pointNum; it < i + 1; it++) {
                    idx_vec.push_back(it);
                    lightpose.position.x = lightXsL[it];
                    lightpose.position.y = lightYsL[it];
                    lightpoints.poses.push_back(lightpose);


                }
                valarray<size_t> idx_val = util::createValarrayFromVector<size_t>(idx_vec);

                valarray<float> cluster_x = lightXsL[idx_val];

                double center_x = cluster_x.sum() / cluster_x.size();

                valarray<float> cluster_y = lightYsL[idx_val];

                double center_y = cluster_y.sum() / cluster_y.size();
                p.x = center_x;
                p.y = center_y;

                ps.push_back(p);
                cout << "x" << p.x << "y" << p.y << std::endl;


                pose.position.x = p.x;
                pose.position.y = p.y;
                msg.poses.push_back(pose);
            }

            pointNum = 0;

        };

    }

    cout << "get board num " << ps.size();
    ROS_INFO("finish scan");
    boardPub.publish(msg);
    pointPub.publish(lightpoints);










}




int main(int argc, char **argv) {



    std::cout << "Hello, World!" << std::endl;
    //init a node
    ros::init(argc, argv, "start_up");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");


    boardPub = nh.advertise<geometry_msgs::PoseArray>("board", 2);
    pointPub = nh.advertise<geometry_msgs::PoseArray>("lightpoint", 2);

    // test xmlrpcvalue
#if 0
    XmlRpc::XmlRpcValue value;
    nh.getParam("board_position", value);
    auto V = util::createVectorFromXmlRpcValue(value);
    ROS_INFO_STREAM(value[0]["x"]);
    cout << V[0]["x"];
#endif
//    ros::Duration(0.5).sleep();

#if 1
    KdTreeCreator kd(nh, nh_private);
//    kd.updateSensor();
#endif

    ros::Rate r(5);
    while (ros::ok()) {
        kd.detectBoard();

        r.sleep();
    }

    return 0;


    //**** topic
    // publish on o topic
    ros::Publisher chat_pub = nh.advertise<node::mytopic>("chat",1);

    util::Listener l(nh, nh_private);

    auto res2 = l.createSubcriber<sensor_msgs::LaserScan>("scan", 2);
    std::shared_ptr<sensor_msgs::LaserScan> data2 = std::get<0>(res2);
    l.getOneMessage("scan", -1);
    cout << data2.get()->header;

    return 0;

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
#if 0
    auto res = l.createSubcriberFilteredTf<node::mytopic>("chat", 2, "map");
    std::shared_ptr<node::mytopic> data = std::get<0>(res);
    auto res2 = l.createSubcriberFilteredTf<sensor_msgs::LaserScan>("scan", 2, "map");
    std::shared_ptr<sensor_msgs::LaserScan> data2 = std::get<0>(res2);
#endif
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
#if 0
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