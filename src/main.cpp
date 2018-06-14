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
#include <kdtree/search.h>

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
namespace node=catkin_startup;



// how to create search tree for flann or nanoann
// given latest baselink
// and a set of points[x,y,yaw]


// for debug
#define debug_pub true

#if debug_pub
ros::Publisher boardPub, pointPub, markerPub;

#endif
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


class BoardFinder {

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


    vector<Position> getBoardPosition();

    vector<Position> xmlToPoints();

public:
    vector<Position> detectBoard();

    bool updateSensor();

    BoardFinder(ros::NodeHandle nh, ros::NodeHandle nh_provate);


    void find();



};


BoardFinder::BoardFinder(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private),
                                                                           l(nh, nh_private) {
//    util::Listener l(nh, nh_private);
#if 0
    auto res = l.createSubcriberFilteredTf<sensor_msgs::LaserScan>("scan", 2, "map");

#endif
#if 1

    auto res = l.createSubcriber<sensor_msgs::LaserScan>("scan", 10);
#endif


    laser_data_ = std::get<0>(res);

    string filename = "/home/waxz/refloc_ws/src/catkin_startup/launch/board.yaml";

    param_ = util::readFile(filename);

//    updateSensor();



//    getLaserPose();
    getBoardPosition();


}

bool BoardFinder::updateSensor() {
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

void BoardFinder::getLaserPose() {

    ROS_INFO("getLaserPose start tf");
//    l.getOneMessage("scan",-1);
    tf::Transform transform;

    transform.setIdentity();

//    l.getTransform("map", "laser", transform);
    tf::poseTFToMsg(transform, laserPose_);
    ROS_INFO_STREAM(laserPose_);
}

vector<Position> BoardFinder::getBoardPosition() {


    updateSensor();
    getLaserPose();
    vector<Position> points = xmlToPoints();
    return points;

//    xmlToPoints();

}

vector<Position> BoardFinder::xmlToPoints() {
    auto visibel_angle = param_["visibel_angle"].As<double>();
    auto visibel_range_ratio = param_["visibel_range_ratio"].As<double>();

    vector<Position> positions;

    positions.clear();

    Position p;


    util::createMapFromXmlRpcValue<double>(value_[0]);
//    return positions;

#if debug_pub

    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
#endif

    ROS_ERROR("get yaml board Num:%d", int(param_["board_position"].Size()));

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
        ROS_INFO("limit: visibel_angle=%.3f,angle_min=%.3f,angle_max=%.3f", visibel_angle, laser_data_.get()->angle_min,
                 laser_data_.get()->angle_max);


        if (condition1 && condition2) {
            positions.push_back(p);

#if debug_pub

            geometry_msgs::Point point;
            point.x = p.x;
            point.y = p.y;

            geometry_msgs::Pose pose;
            tf::poseTFToMsg(util::tf_util::createTransformFromTranslationYaw(point, p.yaw), pose);
            msg.poses.push_back(pose);
#endif

        }

    }
    ROS_INFO("get boarder:%d", int(positions.size()));
#if debug_pub


    markerPub.publish(msg);
#endif


    return positions;

}


vector<Position> BoardFinder::detectBoard() {
    // upddate sensor
    vector<Position> ps;

    ROS_INFO("start scan");
    if (!updateSensor())
        return ps;

    // vector to valarray
    valarray<float> ranges = util::createValarrayFromVector<float>(laser_data_.get()->ranges);
    valarray<float> intensities = util::createValarrayFromVector<float>(laser_data_.get()->intensities);
    if (intensities.size() == 0) {
        ROS_ERROR("scan has no intensities ");
        return ps;
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
        return ps;

    }

    valarray<float> lightXsL = lightXs[std::slice(0, size - 1, 1)];
    valarray<float> lightXsR = lightXs[std::slice(1, size - 1, 1)];

    valarray<float> lightYsL = lightYs[std::slice(0, size - 1, 1)];
    valarray<float> lightYsR = lightYs[std::slice(1, size - 1, 1)];

    valarray<float> distance = sqrt(pow(lightXsR - lightXsL, 2) + pow(lightYsR - lightYsL, 2));


//    for (int i =0;i< distance.size();i++)
//        cout<<distance[i]<<",";


    Position p;
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

        if (distance[i] < neighbor_thresh) {
            pointNum++;
        }
        if (distance[i] > neighbor_thresh || (i == distance.size() - 1 && distance[i] < neighbor_thresh)) {

            if (i == distance.size() - 1) {
                i++;
            }
            double length = sqrt(
                    pow(lightXs[i] - lightXs[i - pointNum], 2) + pow(lightYs[i] - lightYs[i - pointNum], 2));
            //compute length

            if (length > length_thresh) {

                // get index
                vector<size_t> idx_vec;
                for (size_t it = i - pointNum; it < i + 1; it++) {
                    idx_vec.push_back(it);
                    lightpose.position.x = lightXs[it];
                    lightpose.position.y = lightYs[it];
                    lightpoints.poses.push_back(lightpose);


                }
                valarray<size_t> idx_val = util::createValarrayFromVector<size_t>(idx_vec);

                valarray<float> cluster_x = lightXs[idx_val];

                double center_x = cluster_x.sum() / cluster_x.size();

                valarray<float> cluster_y = lightYs[idx_val];

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

#if debug_pub
    boardPub.publish(msg);
    pointPub.publish(lightpoints);
#endif
    return ps;
}


void BoardFinder::find() {
    // get real board position
    vector<Position> realPoints = getBoardPosition();
    // detect board position
    vector<Position> detectPoints = detectBoard();

    // if successful
    if (!realPoints.empty() && detectPoints.empty()) {

        // transform detectPoints to map frame
        vector<geometry_msgs::PointStamped> points;
        geometry_msgs::PointStamped point;
        point.header.frame_id = "laser";
        point.header.stamp = ros::Time::now();
        for (int i = 0; i < detectPoints.size(); i++) {

            point.point.x = detectPoints[i].x;
            point.point.y = detectPoints[i].y;

            points.push_back(point);

        }
        l.tranformPoints(points, "map");


#if 0
        // find match and sort vector
        findNN(realPoints, detectPoints);
        // compute
        computeTagetVector();
        getBasePose();
        updateOdom();
#endif

    }
}

void ll() {
    //get laser pose
    // get near board position
    // detect board
    // compute target pose
    /* 1. transorm detectBoard to map frame
     * 2. sort and match
     * 3. compute target pose
     * */
    // compute target pose and relative position
    // update
}


// user-defined point type
// inherits std::array in order to use operator[]
class Point2d : public std::array<double, 2> {
public:

    // dimension of space (or "k" of k-d tree)
    // KDTree class accesses this member
    static const int DIM = 2;

    // the constructors
    Point2d() {}

    Point2d(double x, double y) {
        (*this)[0] = x;
        (*this)[1] = y;
    }


};


int main(int argc, char **argv) {

    // generate points
    // generate space
    const int width = 500;
    const int height = 500;
    const int npoints = 100;
    std::vector<Point2d> points(npoints);
    for (int i = 0; i < npoints; i++) {
        const int x = rand() % width;
        const int y = rand() % height;
        points[i] = Point2d(x, y);
    }
    // build k-d tree
    kdt::KDTree<Point2d> kdtree(points);
    // generate query (center of the space)
    const Point2d query(0.5 * width, 0.5 * height);

    // nearest neigbor search
    const int idx = kdtree.nnSearch(query);

    // k-nearest neigbors search
    const int k = 10;

    const std::vector<int> knnIndices = kdtree.knnSearch(query, k);
    // radius search
    const double radius = 50;
    const std::vector<int> radIndices = kdtree.radiusSearch(query, radius);

    vector<int> res;
    util::KdTree<Point2d> tree(points);
    res = tree.queryIndex(query, util::SearchMode::radius, radius);


    return 0;



    std::cout << "Hello, World!" << std::endl;
    //init a node
    ros::init(argc, argv, "start_up");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

#if debug_pub
    boardPub = nh.advertise<geometry_msgs::PoseArray>("detectboard", 2);
    pointPub = nh.advertise<geometry_msgs::PoseArray>("lightpoint", 2);
    markerPub = nh.advertise<geometry_msgs::PoseArray>("board", 2);
#endif

#if 0
    util::Listener test(nh, nh_private);


    ros::Duration(1.5).sleep();

    geometry_msgs::PointStamped p,q;
    p.header.frame_id = "laser";
    p.header.stamp=ros::Time::now();
    p.point.x = 1;
    vector<geometry_msgs::PointStamped> points;
    points.push_back(p);
    test.tranformPoints(points,"map");
    cout<<points[0];


    geometry_msgs::PoseStamped poses;
    poses.header =p.header;
    poses.pose.position = p.point;
    poses.pose.orientation.w =1;
    test.tranformPose(poses,"map");
    cout<<poses;
    return 0;
#endif

//    ros::Duration(0.5).sleep();

#if 1
    BoardFinder kd(nh, nh_private);
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