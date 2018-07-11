//
// Created by waxz on 18-7-4.
//

#include "locate_reflection/boardFinder.h"
#include <locate_reflection/patternMatcher.h>
#include <cpp_utils/svdlinefitting.h>


#include <boost/assign.hpp>
#include <opencv2/opencv.hpp>


#define debug_pub true
#define debug_bypass false
BoardFinder::BoardFinder(ros::NodeHandle nh, ros::NodeHandle nh_private) :
        nh_(nh), nh_private_(nh_private), l(nh, nh_private) {

    // **** parameter
    // topic
    scan_topic_ = "scan";
    odomtf_topic_ = "amcl_tf";
    initialpose_topic_ = "initialpose";
    set_particles_service_name_ = "/amcl/set_particles";
    // frame
    odom_frame_id_ = "odom";
    base_frame_id_ = "base_link";
    laser_frame_id_ = "laser";
    fixed_frame_id_ = "map";

    // reset state
    baseLaserTf_.setIdentity();
    mapOdomTf_.setIdentity();
    odomBaseTf_.setIdentity();
    getFirstmapOdomTf_ = false;
    lastPublishOk_ = false;
    matchNum_ = 0;

    // create scan sub
    auto res = l.createSubcriber<sensor_msgs::LaserScan>(scan_topic_, 10);
    // shared scan data
    laser_data_ = std::get<0>(res);

    // create tf sub
    auto res2 = l.createSubcriber<geometry_msgs::PoseWithCovarianceStamped>(odomtf_topic_, 1);
    // shared tf data
    mapOdom_data_ = std::get<0>(res2);

    // create initialpose sub

    auto res3 = l.createSubcriber<geometry_msgs::PoseWithCovarianceStamped>(initialpose_topic_, 1);

    initialPose_data_ = std::get<0>(res3);

    // create /amcl/set_particles
    l.createServiceClient<amcl::amcl_particles>(set_particles_service_name_);


    //  thread shared data
    mapToodomtfPtr_ = std::make_shared<tf::StampedTransform>();

    // read xml
    string filename = "board.yaml";
    try {
        param_ = Yaml::readFile(filename);

    } catch (...) {
        printf("read %s failed!!\n", filename.c_str());
#if debug_bypass
        exit(0);
#endif
    }

    // publish point  for rviz, calibration
    boardPub = nh_.advertise<geometry_msgs::PoseArray>("detectboard", 2);
    pointPub = nh_.advertise<geometry_msgs::PoseArray>("lightpoint", 2);
    markerPub = nh_.advertise<geometry_msgs::PoseArray>("board", 2);


    // point pattern matcher
    patternMatcher = PatternMatcher();

    // threading
    tfb_ = new tf::TransformBroadcaster();
    tfThread_.set(tfb_);
    threadClass_.setTarget(tfThread_, mapToodomtfPtr_);

    // start running in any function
    // threadClass_.start();

    // get mapodomtf as soon as posiible
    // this topic may just publish once
    if (!getMapOdomTf(5)) {
        ROS_ERROR("not received %s;exit", odomtf_topic_.c_str());
#if debug_bypass
        exit(0);
#endif
    }

}

BoardFinder::~BoardFinder() {
    delete tfb_;
}

// get intialpose
// we only know it's seted, and wait for new amcl_tf
bool BoardFinder::getInitialpose() {
    bool getmsg = l.getOneMessage(initialpose_topic_, 0.1);
    return getmsg;
}

// get scan data
// if no data , return false;
bool BoardFinder::updateSensor() {
    bool getmsg = l.getOneMessage(scan_topic_, 0.1);
    size_t size = laser_data_.get()->ranges.size();
    if (bear_.size() != size) {
        bear_ = valarray<float>(0.0, size);
        for (int i = 0; i < size; i++) {
            bear_[i] = laser_data_.get()->angle_min + i * laser_data_.get()->angle_increment;
        }
    }
    bool getpose = getLaserPose();

    return getmsg && getpose;
}

// get tf data
// if no data return false;
bool BoardFinder::getMapOdomTf(int sleep) {

    // check if get initialpose

    bool getresetpose = getInitialpose();
    // wait amcl tf from the first
    // at reboot, this tf only publish once
    bool getmsg;
    if (getresetpose || (lastPublishOk_ && matchNum_ == 0)) {
        getmsg = l.getOneMessage(odomtf_topic_, -1);

    } else {
        getmsg = l.getOneMessage(odomtf_topic_, sleep);

    };

    // todo:baypass
#if debug_bypass
    if (!getFirstmapOdomTf_) {
        mapOdomTf_.setOrigin(tf::Vector3(-18.659, 13.62, 0.0));
        mapOdomTf_.setRotation(tf::Quaternion(0.0, 0.0, -0.64837, 0.76133));
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(mapOdomTf_, pose);
        mapOdom_data_.get()->pose.pose = pose;
        getmsg = true;
    }

#endif

    if (!getFirstmapOdomTf_ || getresetpose) {
        if (!getmsg) {
            return false;
        } else {
            getFirstmapOdomTf_ = true;
            tf::Transform transform;
            tf::poseMsgToTF(mapOdom_data_.get()->pose.pose, transform);
            // shared and start publishing in thread
            updateSharedData(transform);
        }
    } else {
        // get tf at usual work time
        // if last match succefful; not update local tf

        if (!lastPublishOk_ && getmsg) {
            // detect fail at some condition, must reset amcl using last correct odom, and use new comming data to update local odom
            // update local mapodomtf
            tf::poseMsgToTF(mapOdom_data_.get()->pose.pose, mapOdomTf_);

        }

    }

    // not recieve data?
    ROS_INFO_STREAM("==== mapodom\n" << mapOdom_data_.get()->pose.pose);
    return true;
}

// get laser pose
// store to laserPose_
bool BoardFinder::getLaserPose() {

    ROS_INFO("getLaserPose start tf");
    if (!getMapOdomTf()) {
        ROS_INFO("no amcl tf ;skip");
        return false;
    }
//    l.getOneMessage("scan",-1);
    tf::Transform transform;

    transform.setIdentity();

    // get odom to laser tf
    bool succ = l.getTransform(odom_frame_id_, laser_frame_id_, transform, laser_data_.get()->header.stamp, 0.01,
                               false);
    // get laser pose base on mapodomtf and odomtolasertf
    if (succ) {
        tf::poseTFToMsg(mapOdomTf_ * transform, laserPose_);
        ROS_INFO_STREAM("laser pose\n " << laserPose_);
    }
    // get odom to base tf
    bool succ2 = l.getTransform(odom_frame_id_, base_frame_id_, odomBaseTf_, laser_data_.get()->header.stamp, 0.01,
                                false);

    if (baseLaserTf_.getOrigin().getX() == 0.0)
        l.getTransform(base_frame_id_, laser_frame_id_, baseLaserTf_, laser_data_.get()->header.stamp);

    return succ && succ2;
}

bool BoardFinder::getBoardPosition(vector<Position> &pointsW, vector<Position> &points) {


    // read xml file
    xmlToPoints(pointsW);
    points = pointsW;
    // transform point from map frame to laser frame
    transformPoints(points);

    return true;

//    xmlToPoints();

}

// read position from xml file
// sort order in clockwise order; do this in transorm function
bool BoardFinder::xmlToPoints(vector<Position> &pointsW) {
    auto visibel_angle = param_["visibel_angle"].As<double>();
    auto visibel_range_ratio = param_["visibel_range_ratio"].As<double>();


    pointsW.clear();

    Position p;


//    return positions;

#if debug_pub

    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
#endif

    ROS_ERROR("get yaml board Num:%d", int(param_["board_position"].Size()));

    int size = param_["board_position"].Size();
    double start_angle = 0;
    for (int it = 0; it < size; it++) {

        p.x = param_["board_position"][it]["x"].As<double>();

        p.y = param_["board_position"][it]["y"].As<double>();

        p.yaw = param_["board_position"][it]["yaw"].As<double>();

        p.length = param_["board_position"][it]["length"].As<double>();



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

        p.angle = normalAngle(robottoboardangle, yaw);
#if 1
        // fix 180 degree turing bug
        if (it == 0) {
            start_angle = p.angle;
            p.angle = 0.0;
        } else {
            p.angle = normalAngle(p.angle, start_angle);
        }
#endif
//        bool in_view = robottoboardangle > laser_data_.get()->angle_min && robottoboardangle < laser_data_.get()->angle_max;
        double direction1 = normalDiff(boardtorobotangle, p.yaw);
        double direction2 = normalDiff(robottoboardangle, yaw);
        bool condition2 = direction2 > laser_data_.get()->angle_min && direction2 < laser_data_.get()->angle_max;
        bool condition1 = direction1 < visibel_angle;

        ROS_INFO("data inrange,%.3f,%.3f,%.3f,%.3f", robottoboardangle, boardtorobotangle, direction1, direction2);
        ROS_INFO("limit: visibel_angle=%.3f,angle_min=%.3f,angle_max=%.3f", visibel_angle, laser_data_.get()->angle_min,
                 laser_data_.get()->angle_max);


        if (condition1 && condition2) {
            pointsW.push_back(p);

#if debug_pub

            geometry_msgs::Point point;
            point.x = p.x;
            point.y = p.y;

            geometry_msgs::Pose pose;
            tf::poseTFToMsg(tf_util::createTransformFromTranslationYaw(point, p.yaw), pose);
            msg.poses.push_back(pose);
#endif

        }

    }
    ROS_INFO("get boarder:%d", int(pointsW.size()));
#if debug_pub


    markerPub.publish(msg);
#endif


    // sort points
    std::sort(pointsW.begin(), pointsW.end(), angleCompare);

    return true;

}


vector<Position> BoardFinder::detectBoard() {
    // upddate sensor
    vector<Position> ps;

    ROS_INFO("start scan");

    // vector to valarray
    valarray<float> ranges = container::createValarrayFromVector<float>(laser_data_.get()->ranges);
    valarray<float> intensities = container::createValarrayFromVector<float>(laser_data_.get()->intensities);
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

    // point neighbor threash = num*bean_angle*distance;
    neighbor_thresh = neighbor_thresh * laser_data_.get()->angle_increment;

//    valarray<float> lightPoitns = intensities[intensities>intensity_thresh];
    valarray<float> lightXs = xs[intensities > intensity_thresh];
    valarray<float> lightYs = ys[intensities > intensity_thresh];
    valarray<float> rangesMask = ranges[intensities > intensity_thresh];

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
        double thresh = neighbor_thresh * rangesMask[i];
//        thresh = 0.5;

        double d = distance[i];
        if (d < thresh) {

            pointNum++;
        }
        if (d > thresh || (i == distance.size() - 1 && d < thresh)) {

            if (i == distance.size() - 1) {
                i++;
            }
            double length = sqrt(
                    pow(lightXs[i] - lightXs[i - pointNum], 2) + pow(lightYs[i] - lightYs[i - pointNum], 2));
            //compute length

            if (length > length_thresh) {

                // get index
                vector<size_t> idx_vec;
                // push light point to vector
                // fit line
                vector<cv::Point2d> FitPoints;
                for (size_t it = i - pointNum; it < i + 1; it++) {
                    idx_vec.push_back(it);
                    lightpose.position.x = lightXs[it];
                    lightpose.position.y = lightYs[it];
                    lightpoints.poses.push_back(lightpose);
                    FitPoints.push_back(cv::Point2f(lightXs[it], lightYs[it]));

                }
                //call rnasac fit
                //double angle = LineFitting(FitPoints);

                // call svd line fit
                double angle = svdfit(FitPoints);



                valarray<size_t> idx_val = container::createValarrayFromVector<size_t>(idx_vec);

                valarray<float> cluster_x = lightXs[idx_val];

                double center_x = cluster_x.sum() / cluster_x.size();

                valarray<float> cluster_y = lightYs[idx_val];

                double center_y = cluster_y.sum() / cluster_y.size();

                // angle shoud point to robot
                double angletorobot = atan2(-center_y, -center_x);

                if (normalDiff(angletorobot, angle + 0.5 * M_PI) < normalDiff(angletorobot, angle - 0.5 * M_PI)) {
                    angle += 0.5 * M_PI;
                } else {
                    angle -= 0.5 * M_PI;

                }

                p.x = center_x;
                p.y = center_y;
                p.yaw = angle;

                ps.push_back(p);
                cout << "x" << p.x << "y" << p.y << "angle" << p.yaw << "length:" << length << std::endl;


                pose.position.x = p.x;
                pose.position.y = p.y;
                pose.orientation = tf::createQuaternionMsgFromYaw(angle);
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

bool BoardFinder::transformPoints(vector<Position> &realPoints) {
    // get odom to laser transform
    tf::Transform mapLasertf;
    tf::poseMsgToTF(laserPose_, mapLasertf);


    geometry_msgs::Point point;
#if debug_pub

    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser";
#endif

    for (int i = 0; i < realPoints.size(); i++) {

        tf::Point p(realPoints[i].x, realPoints[i].y, 0.0);
        tf::pointTFToMsg(mapLasertf.inverse() * p, point);

        realPoints[i].y = point.y;
        realPoints[i].x = point.x;

        geometry_msgs::Pose pose;
        tf::poseTFToMsg(tf_util::createTransformFromTranslationYaw(point, 0.0), pose);
        msg.poses.push_back(pose);
    }

    // sort

    markerPub.publish(msg);

    return true;
}


bool
BoardFinder::findNN(vector<Position> &realPointsW, vector<Position> &realPoints, vector<Position> &detectPoints) {


    // if point Num >=2
    // use pattern matncher
    // if only detect one ;
    // use kdtree search

    vector<Position> realPointReg, detecctPointReg;

    if (detectPoints.size() > 1) {
        // call pattern match
        // how to detect failure
        auto assign = patternMatcher.match(detectPoints, realPoints);
        if (assign.empty()) {

            cout << "match failure!!" << endl;
            realPointsW = realPointReg;
            detectPoints = detecctPointReg;
            return false;

        }

        for (int i = 0; i < assign.size(); i++) {
            realPointReg.push_back(realPointsW[std::get<1>(assign[i])]);
            detecctPointReg.push_back(detectPoints[std::get<0>(assign[i])]);
        }
        realPointsW = realPointReg;
        detectPoints = detecctPointReg;

        return true;
    } else {

        // create kd tree
        // search
        // create kdtree
        const int npoints = realPoints.size();
        std::vector<kdtree::Point2d> points(npoints);
        for (int i = 0; i < npoints; i++) {
            points[i] = kdtree::Point2d(realPoints[i].x, realPoints[i].y);
        }


        // build k-d tree
        kdtree::KdTree<kdtree::Point2d> kdtree(points);

        double radius = param_["query_radius"].As<double>();;
        // query point
        vector<vector<int> > results;
        for (int i = 0; i < detectPoints.size(); i++) {
            // create query point
            //search
            vector<int> res;


            kdtree::Point2d query(detectPoints[i].x, detectPoints[i].y);
            res = kdtree.queryIndex(query, kdtree::SearchMode::radius, radius);

            results.push_back(res);
        }

        if (results.empty()) {
            return false;
        }
        for (int i = 0; i < results.size(); i++) {
            if (!results[i].empty()) {

                detecctPointReg.push_back(detectPoints[i]);
                realPointReg.push_back(realPointsW[results[i][0]]);
            }
        }

        realPointsW = realPointReg;
        detectPoints = detecctPointReg;
        return true;

        // check results, remove duplicate or invalid points;


        // start with a point matcn only one point
        // maybe it's fake detection


        // clear and check distancce  and sort vector

    }

#if 0


#endif
}

// use laser pose to update mapodomtf
void BoardFinder::updateMapOdomTf(tf::Transform laserPose) {
    geometry_msgs::Pose debug_pose;
    tf::poseTFToMsg(laserPose, debug_pose);


    updateSharedData(laserPose * baseLaserTf_.inverse() * odomBaseTf_.inverse());



//    l.sendTransform("map", "odom", mapTOodomTf, 0.1);


}

void BoardFinder::updateSharedData(tf::Transform mapTOodomTf) {

    ros::Time tn = ros::Time::now();
    ros::Duration transform_tolerance;
    transform_tolerance.fromSec(0.1);
    ros::Time transform_expiration = (tn + transform_tolerance);
    mapOdomTf_ = tf::StampedTransform(mapTOodomTf,
                                      transform_expiration,
                                      fixed_frame_id_, odom_frame_id_);

    geometry_msgs::Pose debug_odom;
    tf::poseTFToMsg(mapOdomTf_, debug_odom);
    ROS_ERROR_STREAM("update odom \n" << debug_odom);


    // segementation  fault: use shared ptr before initialise
    tf::StampedTransform tmp(mapOdomTf_);
    std::swap(*mapToodomtfPtr_, tmp);

    // if thread is not running ; start running
    if (!threadClass_.isRunning()) {
#if 1
        threadClass_.start();
#endif
    }
}

void BoardFinder::computeUpdatedPose(vector<Position> realPoints, vector<Position> detectPoints) {


    geometry_msgs::Point translation;
    double realX = 0, realY = 0, detectX = 0, detectY = 0;
    double realyaw, detectyaw;
    tf::Transform realTarget, detectTarget, laserPose;


    if (detectPoints.size() > 1) {
        // get points pairs
        // compute target vector
        int size = realPoints.size();
        for (int i = 0; i < size; i++) {
            realX += realPoints[i].x;
            realY += realPoints[i].y;
            detectX += detectPoints[i].x;
            detectY += detectPoints[i].y;

        }
        realX /= size;
        realY /= size;
        detectX /= size;
        detectY /= size;


        realyaw = std::atan2(realPoints[0].y - realPoints[size - 1].y, realPoints[0].x - realPoints[size - 1].x);
        detectyaw = std::atan2(detectPoints[0].y - detectPoints[size - 1].y,
                               detectPoints[0].x - detectPoints[size - 1].x);
    } else {
        realX = realPoints[0].x;
        realY = realPoints[0].y;
        realyaw = realPoints[0].yaw;
        detectX = detectPoints[0].x;
        detectY = detectPoints[0].y;
        detectyaw = detectPoints[0].yaw;

    }

    translation.x = realX;
    translation.y = realY;

    realTarget = tf_util::createTransformFromTranslationYaw(translation, realyaw);
    translation.x = detectX;
    translation.y = detectY;
    detectTarget = tf_util::createTransformFromTranslationYaw(translation, detectyaw);


    //compute laser pose

    laserPose = realTarget * detectTarget.inverse();

    geometry_msgs::Pose debug_pose;
    tf::poseTFToMsg(laserPose, debug_pose);
    ROS_ERROR_STREAM("laser pose \n" << debug_pose);





    // compute base_link pose


    // get odom-baselink

    updateMapOdomTf(laserPose);


    // compute map-odom


}

// reset amcl given base pose
bool BoardFinder::resetAmcl() {
    using namespace Eigen;
    amcl::amcl_particles srv;

    // add pse array
    srv.request.pose_array_msg.poses.clear();
//    srv.request.pose_array_msg.poses.push_back(latest_pose);

    // add initial pose
    geometry_msgs::PoseWithCovariance initial_pose;
    // set mapodomtf*odombasetf as initial pose
    tf::poseTFToMsg(mapOdomTf_ * odomBaseTf_, initial_pose.pose);

    double yaw = tf::getYaw(initial_pose.pose.orientation);

    // cov
    auto x_cov = param_["x_cov"].As<double>();
    auto y_cov = param_["y_cov"].As<double>();
    auto yaw_cov = param_["yaw_cov"].As<double>();


    MatrixXd mat_1(2, 2);
    mat_1 << cos(yaw), cos(yaw + 0.5 * M_PI),
            sin(yaw), sin(yaw + 0.5 * M_PI);
#if 1
    MatrixXd mat_2(2, 2);
    mat_2 << x_cov, 0,
            0, y_cov;

    MatrixXd mat_3(2, 2);
    mat_3 = mat_1 * mat_2 * mat_1.transpose();

    initial_pose.covariance = boost::assign::list_of
            (mat_3(0, 0))(mat_3(0, 1))(0)(0)(0)(0)
            (mat_3(1, 0))(mat_3(1, 1))(0)(0)(0)(0)
            (0)(0)(0)(0)(0)(0)
            (0)(0)(0)(0)(0)(0)
            (0)(0)(0)(0)(0)(0)
            (0)(0)(0)(0)(0)(yaw_cov);
#endif
    srv.request.initial_pose.header.stamp = laser_data_.get()->header.stamp;
    srv.request.initial_pose.header.frame_id = "map";
    srv.request.initial_pose.pose = initial_pose;


    ROS_INFO("set filter cnt: %d", int(srv.request.pose_array_msg.poses.size()));


    if (l.callService(set_particles_service_name_, srv)) {
        ROS_INFO("locate call service ok!: %ld", (long int) srv.response.success);
    } else {
        ROS_ERROR("locate Failed to call service set_filter");
    }

    return true;


}

// find reflection board and update map-odom tf
// get board in map ; given baselink
// get board from scan
// match two point pattern
// compute relative pose
// update tf
void BoardFinder::findLocation() {
    // get real board position
    vector<Position> realPointsW;
    vector<Position> realPoints;
    // get scan data
    if (!updateSensor()) {

        ROS_INFO("No laser, skip!!");
        return;
    }

    // get board from xml data
    getBoardPosition(realPointsW, realPoints);


    // detect board position
    vector<Position> detectPoints = detectBoard();

    // test match function
#if 0
    // test matcher;ok
    vector<Position> p1, p2;
    p1.push_back(Position(-1, 2, 3.3,1));
    p1.push_back(Position(1, 2, 3.3,1));
    p1.push_back(Position(1.5, 1.5, 3.3,1));

    p2.push_back(Position(-3.1, 1.9, 3.3,1));
    p2.push_back(Position(-1.1, 1.9, 3.3,1));
    p2.push_back(Position(0.9, 1.9, 3.3,1));
    p2.push_back(Position(1.4, 1.4, 3.3,1));
    realPoints = p2;
    realPointsW = p2;
    detectPoints = p1;
#endif


    // if successful
    if (!realPoints.empty() && !detectPoints.empty()) {

        // find matched pattern
        findNN(realPointsW, realPoints, detectPoints);

        // if detect more board
        // 1 board: compute with position and yaw
        // 2 and more , compute with relative position of betwwen board

        if (!detectPoints.empty()) {
            // compute target vector
            computeUpdatedPose(realPointsW, detectPoints);
#if 1
            int resetDuration_ = param_["resetDuration"].As<int>(5);

            if (matchNum_ % resetDuration_ == 0) {
                resetAmcl();

                if (matchNum_ > 0) {
                    matchNum_ = 0;
                }
            }
            matchNum_++;

#endif
            lastPublishOk_ = true;

        } else {
            // find no match board
            tf::Transform transform;
            tf::poseMsgToTF(laserPose_, transform);
            updateMapOdomTf(transform);
            lastPublishOk_ = false;
        }


    } else {
        // detect no board
        tf::Transform transform;
        tf::poseMsgToTF(laserPose_, transform);
        updateMapOdomTf(transform);
        lastPublishOk_ = false;

    }
}



