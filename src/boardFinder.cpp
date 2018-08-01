//
// Created by waxz on 18-7-4.
//

#include "locate_reflection/boardFinder.h"
#include <locate_reflection/patternMatcher.h>
#include <cpp_utils/svdlinefitting.h>


#include <boost/assign.hpp>
#include <opencv2/opencv.hpp>

#define log_lv1  false
#define debug_pub true
#define debug_bypass false
BoardFinder::BoardFinder(ros::NodeHandle nh, ros::NodeHandle nh_private) :
        nh_(nh), nh_private_(nh_private), l(nh, nh_private) {
    initParams();

    // **** parameter
    // topic
    scan_topic_ = "scan";
    odomtf_topic_ = "amcl_tf";
    locate_tf_topic_ = "locate_tf";
    initialpose_topic_ = "initialpose";
    set_particles_service_name_ = "/amcl/set_particles";
    vel_topic_ = "vel_actual";
    // frame
    odom_frame_id_ = "odom";
    base_frame_id_ = "base_link";
    laser_frame_id_ = "laser";
    fixed_frame_id_ = "map";
    if (!nh_private_.getParam("use_locate", use_locate_))
        use_locate_ = false;

    // reset state
    baseLaserTf_.setIdentity();
    mapOdomTf_.setIdentity();
    odomBaseTf_.setIdentity();
    getFirstmapOdomTf_ = false;
    lastPublishOk_ = false;
    matchNum_ = 1;
    detectOne_ = false;
    getresetpose_ = false;

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

    // create locate_tf sub
    auto res4 = l.createSubcriber<geometry_msgs::PoseWithCovarianceStamped>(locate_tf_topic_, 1);

    locate_odom_data_ = std::get<0>(res4);

    // create /amcl/set_particles
    l.createServiceClient<amcl::amcl_particles>(set_particles_service_name_);

    // create vel_actual_data_
    auto res5 = l.createSubcriber<geometry_msgs::Twist>(vel_topic_, 1);
    vel_actual_data_ = std::get<0>(res5);


    //  thread shared data
    mapToodomtfPtr_ = std::make_shared<tf::StampedTransform>();

    // read xml
    string filename = "board.yaml";
    try {
        param_ = Yaml::readFile(filename);

    } catch (...) {
        printf("read %s failed!!\n", filename.c_str());
        exit(0);
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

void BoardFinder::initParams() {

    query_radius_ = param_["query_radius"].As<double>(1.5);
    distScoreBase_ = param_["distScoreBase"].As<double>(0.1);
    angleScoreBase_ = param_["angleScoreBase"].As<double>(0.1);
    distRatio_ = param_["distRatio"].As<double>(2);
    angleRatio_ = param_["angleRatio"].As<double>(6);
    matchScore_ = param_["matchScore"].As<double>(1.1);
    finalScore_ = param_["finalScore"].As<double>(1.1);
    static_dist_ = param_["static_dist"].As<double>(0.05);
    static_angle_ = param_["static_angle"].As<double>(0.02);
    vel_angular_min_ = param_["vel_angular_min"].As<double>(0.3);

    amcl_diff_max_ = param_["amcl_diff_max"].As<double>(6);
}

// get intialpose
// we only know it's seted, and wait for new amcl_tf
bool BoardFinder::getInitialpose() {
    bool getmsg = l.getOneMessage(initialpose_topic_, 0.01);
    return getmsg;
}

// get vel
bool BoardFinder::getVel() {
    bool getmsg = l.getOneMessage(vel_topic_, 0.01);
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

    bool get_rotation = getVel();
    bool rotation_fast_ = false;
    if (get_rotation) {
        rotation_fast_ = fabs((*vel_actual_data_).angular.z) > vel_angular_min_;
    }

    return getmsg && getpose && !rotation_fast_;
}

// get tf data
// if no data return false;
bool BoardFinder::getMapOdomTf(double sleep) {

    // check if get initialpose

    getresetpose_ = getInitialpose();
    // wait amcl tf from the first
    // at reboot, this tf only publish once
    bool getmsg;
    bool get_locate_msg = false;
    if (getresetpose_) {
        getmsg = l.getOneMessage(odomtf_topic_, -1);

    } else {
        getmsg = l.getOneMessage(odomtf_topic_, sleep);

        if (use_locate_) {
            get_locate_msg = l.getOneMessage(locate_tf_topic_, sleep);
        }

    };
    if (lastPublishOk_ && matchNum_ == 0) {
        if (!getmsg) {
            return false;
        }
    }


    if (!getFirstmapOdomTf_ || getresetpose_) {
        if (!getmsg) {
            return false;
        } else {
            // get fisr tf
            getFirstmapOdomTf_ = true;
            tf::Transform transform;
            tf::poseMsgToTF(mapOdom_data_.get()->pose.pose, transform);
            // shared and start publishing in thread
            updateSharedData(transform);
        }
    } else {
        // get tf at usual work time
        // if last match succefful; not update local tf

        if (!lastPublishOk_) {
            // detect fail at some condition, must reset amcl using last correct odom, and use new comming data to update local odom
            // update local mapodomtf
            if (getmsg && !use_locate_) {
                geometry_msgs::Pose debug_pose;
                debug_pose = mapOdom_data_.get()->pose.pose;
                tf::poseMsgToTF(mapOdom_data_.get()->pose.pose, mapOdomTf_);
            }
            if (get_locate_msg && use_locate_) {
                geometry_msgs::Pose debug_pose;
                debug_pose = locate_odom_data_.get()->pose.pose;
                tf::poseMsgToTF(locate_odom_data_.get()->pose.pose, mapOdomTf_);

            }



        }

    }

    // not recieve data?
#if log_lv1
    ROS_INFO_STREAM("==== mapodom\n" << mapOdom_data_.get()->pose.pose);
#endif
    return true;
}

// get laser pose
// store to laserPose_
bool BoardFinder::getLaserPose() {
    if (baseLaserTf_.getOrigin().getX() == 0.0)
        l.getTransform(base_frame_id_, laser_frame_id_, baseLaserTf_, laser_data_.get()->header.stamp, 0.1, true);
#if log_lv1

    ROS_INFO("getLaserPose start tf");
#endif
    if (!getMapOdomTf(0.05)) {
        ROS_INFO("no amcl tf ;skip");
        return false;
    }
//    l.getOneMessage("scan",-1);
#if 0
    tf::Transform transform;

    transform.setIdentity();
#endif
    // get odom to laser tf
    bool succ = l.getTransform(odom_frame_id_, base_frame_id_, odomBaseTf_, laser_data_.get()->header.stamp, 0.05,
                               false);
    // get laser pose base on mapodomtf and odomtolasertf
    if (succ) {
        geometry_msgs::Pose laserPose;
        tf::poseTFToMsg(mapOdomTf_ * odomBaseTf_ * baseLaserTf_, laserPose);
        // get invalid data? from which source
#if log_lv1
        geometry_msgs::Pose p1, p2, p3;
        tf::poseTFToMsg(mapOdomTf_, p1);
        tf::poseTFToMsg(odomBaseTf_, p2);
        tf::poseTFToMsg(baseLaserTf_, p3);
        ROS_ERROR_STREAM("mapOdomTf_\n" << p1);
        ROS_ERROR_STREAM("odomBaseTf_\n" << p2);
        ROS_ERROR_STREAM("baseLaserTf_\n" << p3);
        ROS_ERROR_STREAM("laserPose\n" << laserPose);
#endif
        // todo:debug update laser pose
        if (getresetpose_) {
            getresetpose_ = false;
        }
#if 0
        if(laserPose.position.x>17.5 || laserPose.position.x<16.5){
            ROS_ERROR_STREAM("exit at last laserPose\n"<<laserPose_);

            exit(0);
        }
#endif
        laserPose_ = laserPose;

#if log_lv1

        ROS_INFO_STREAM("laser pose\n " << laserPose_);
#endif
    }
    // get odom to base tf
#if 0
    bool succ2 = l.getTransform(odom_frame_id_, base_frame_id_, odomBaseTf_, laser_data_.get()->header.stamp, 0.01,
                                false);
#endif


    return succ;
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
    auto visibel_angle = param_["visibel_angle"].As<double>(1.4);
    auto visibel_range_ratio = param_["visibel_range_ratio"].As<double>(0.4);


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
#if log_lv1

        ROS_INFO("board %.3f,%.3f,%.3f,%.3f,%.3f,%.3f", p.x, p.y, p.yaw, laserPose_.position.x, laserPose_.position.y,
                 yaw);
#endif
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
#if log_lv1

        ROS_INFO("data inrange,%.3f,%.3f,%.3f,%.3f", robottoboardangle, boardtorobotangle, direction1, direction2);
        ROS_INFO("limit: visibel_angle=%.3f,angle_min=%.3f,angle_max=%.3f", visibel_angle, laser_data_.get()->angle_min,
                 laser_data_.get()->angle_max);
#endif

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

    for (int i = 0; i < pointsW.size(); i++) {
        printf("get board in map [%.3f,%.3f,%.3f]", pointsW[i].x, pointsW[i].y, pointsW[i].yaw);
        std::cout << std::endl;
    }

    return true;

}


vector<Position> BoardFinder::detectBoard() {
    // upddate sensor
    vector<Position> ps;

#if 0
    if (!updateSensor()) {

        ROS_INFO("No laser, skip!!");
        return ps;
    }
#endif
#if log_lv1

    ROS_INFO("start scan");
#endif
    // vector to valarray
    valarray<float> ranges = container::createValarrayFromVector<float>(laser_data_.get()->ranges);
    valarray<float> intensities = container::createValarrayFromVector<float>(laser_data_.get()->intensities);
    if (intensities.size() == 0) {
        ROS_ERROR("scan has no intensities ");
        exit(0);
        return ps;

    }

    valarray<float> xs = ranges * cos(bear_);
    valarray<float> ys = ranges * sin(bear_);


    auto intensity_thresh = param_["intensity_thresh"].As<float>(600);
    auto neighbor_thresh = param_["neighbor_thresh"].As<float>(5);
    auto length_thresh = param_["length_thresh"].As<float>(0.03);

    // point neighbor threash = num*bean_angle*distance;
    neighbor_thresh = neighbor_thresh * laser_data_.get()->angle_increment;

//    valarray<float> lightPoitns = intensities[intensities>intensity_thresh];
    valarray<float> lightXs = xs[intensities > intensity_thresh];
    valarray<float> lightYs = ys[intensities > intensity_thresh];
    valarray<float> rangesMask = ranges[intensities > intensity_thresh];

    size_t size = lightXs.size();
    if (size == 0) {
#if log_lv1

        ROS_ERROR("no light point");
#endif
        return ps;

    }

    valarray<float> lightXsL = lightXs[std::slice(0, size - 1, 1)];
    valarray<float> lightXsR = lightXs[std::slice(1, size - 1, 1)];

    valarray<float> lightYsL = lightYs[std::slice(0, size - 1, 1)];
    valarray<float> lightYsR = lightYs[std::slice(1, size - 1, 1)];

    valarray<float> distance = sqrt(pow(lightXsR - lightXsL, 2) + pow(lightYsR - lightYsL, 2));

    valarray<float> rangesMaskL = rangesMask[std::slice(0, size - 1, 1)];


//    for (int i =0;i< distance.size();i++)
//        cout<<distance[i]<<",";


    Position p;
    size_t pointNum = 0;
    double pointLength = 0;

    //publish pose
    geometry_msgs::PoseArray msg, lightpoints;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    lightpoints.header.stamp = ros::Time::now();
    lightpoints.header.frame_id = "laser";
    geometry_msgs::Pose pose, lightpose;

    // fix: board at -180degree, mistaking split board

    int start_idx = 0;
    int sz = distance.size();
    double dist_max = distance.max();

    if (dist_max > 0.1) {
        for (start_idx = 0; start_idx < sz; start_idx++) {
            if (distance[start_idx] == distance.max()) {
                break;
            }
        }
        // cshift array
        lightXs = lightXs.cshift(start_idx + 1);
        lightYs = lightYs.cshift(start_idx + 1);
        rangesMask = rangesMask.cshift(start_idx + 1);


        // recompute distance
        lightXsL = lightXs[std::slice(0, size - 1, 1)];
        lightXsR = lightXs[std::slice(1, size - 1, 1)];

        lightYsL = lightYs[std::slice(0, size - 1, 1)];
        lightYsR = lightYs[std::slice(1, size - 1, 1)];

        distance = sqrt(pow(lightXsR - lightXsL, 2) + pow(lightYsR - lightYsL, 2));
    }


    for (int i = 0; i < sz; i++) {
        double thresh = neighbor_thresh * rangesMask[i];
//        thresh = 0.5;

        double d = distance[i];
        if (d < thresh) {

            pointNum++;
        }
        if (d > thresh || (i == sz - 1)) {
#if 1
            if (i == sz - 1) {
                i++;
            }
#endif
            double length = sqrt(
                    pow(lightXs[i] - lightXs[i - pointNum], 2) + pow(lightYs[i] - lightYs[i - pointNum], 2));
            //compute length

            if (length > length_thresh) {

                // get index
                vector<size_t> idx_vec;
                // push light point to vector
                // fit line
                vector<cv::Point2d> FitPoints;
                for (size_t it = i - pointNum; it < std::min(i + 1, sz - 1); it++) {
                    idx_vec.push_back(it);
                    lightpose.position.x = lightXs[it];
                    lightpose.position.y = lightYs[it];
                    lightpoints.poses.push_back(lightpose);
                    FitPoints.push_back(cv::Point2f(lightXs[it], lightYs[it]));

                }
                //call rnasac fit
                //double angle = LineFitting(FitPoints);

                // call svd line fit
                double angle = 0.0;


                bool fitted = FitPoints.size() > 1;
                if (fitted)
                    angle = svdfit(FitPoints);
                else {
                    continue;
                }



                valarray<size_t> idx_val = container::createValarrayFromVector<size_t>(idx_vec);

                valarray<float> cluster_x = lightXs[idx_val];

                double center_x = cluster_x.sum() / cluster_x.size();

                valarray<float> cluster_y = lightYs[idx_val];

                double center_y = cluster_y.sum() / cluster_y.size();

                // angle shoud point to robot
                double angletorobot = atan2(-center_y, -center_x);

                double board_length = sqrt(pow(cluster_x[0] - cluster_x[cluster_x.size() - 1], 2) +
                                           pow(cluster_y[0] - cluster_y[cluster_y.size() - 1], 2));

                if (fitted) {
                    if (normalDiff(angletorobot, angle + 0.5 * M_PI) < normalDiff(angletorobot, angle - 0.5 * M_PI)) {
                        angle += 0.5 * M_PI;
                    } else {
                        angle -= 0.5 * M_PI;

                    }
                } else {
                    angle = angletorobot;
                };


                p.x = center_x;
                p.y = center_y;
                p.yaw = angle;
                p.length = board_length;

                ps.push_back(p);
                // length > 8m?
                cout << "x" << p.x << "y" << p.y << "angle" << p.yaw << "length:" << p.length << std::endl;


                pose.position.x = p.x;
                pose.position.y = p.y;
                pose.orientation = tf::createQuaternionMsgFromYaw(angle);
                tf::Transform posetf_laser;
                tf::poseMsgToTF(pose,posetf_laser);
                tf::poseTFToMsg(mapOdomTf_*odomBaseTf_*baseLaserTf_*posetf_laser,pose);
                msg.poses.push_back(pose);
            }

            pointNum = 0;

        };

    }

    cout << "get board num " << ps.size();
#if log_lv1

    ROS_INFO("finish scan");
#endif
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

vector<tuple<int, int> > BoardFinder::kdTreeMatch(vector<Position> detectPoints, vector<Position> realPoints) {

    vector<tuple<int, int> > assignments;
    assignments.clear();



    // create kd tree
    // search
    // create kdtree
    const int npoints = realPoints.size();
    std::vector<kdtree::Point2d> points;
    for (int i = 0; i < npoints; i++) {
        points.push_back(kdtree::Point2d(realPoints[i].x, realPoints[i].y));
    }







    // build k-d tree
    kdtree::KdTree<kdtree::Point2d> kdtree(points);

    // query point
    vector<vector<int> > results;
    for (int i = 0; i < detectPoints.size(); i++) {
        // create query point
        //search
        vector<int> res;


        kdtree::Point2d query(detectPoints[i].x, detectPoints[i].y);
        res = kdtree.queryIndex(query, kdtree::SearchMode::radius, query_radius_);

        results.push_back(res);
    }

    if (results.empty()) {
        return assignments;
    }

    // add zero point
    detectPoints.push_back(Position(0, 0, 0, 0, 0));
    realPoints.push_back(Position(0, 0, 0, 0, 0));
    vector<int> res;
    res.push_back(realPoints.size() - 1);
    results.push_back(res);

    // add pattern match
    // 1 ============

    for (int i = 0; i < detectPoints.size() - 1; i++) {
        if (!results[i].empty()) {

            // for each point in result i
            int best_id_i = 0;
            int best_match_i = 0;
            double best_score_i = 0.0;

            // 2 ============

            for (int m = 0; m < results[i].size(); m++) {

                double score_i = 0;
                int match_j = 0;



                // check all other result
                // get i's point to other result's point with least error
                // get error < thresh
                // get mean least error
                // 3 ============

                for (int j = 0; j < results.size(); j++) {
                    // skip empty and j=i
                    if (j == i)
                        continue;

                    double best_score_j = 0.0;
                    if (!results[j].empty()) {
                        //for each point in result[j]
                        // 4 ============

                        for (int k = 0; k < results[j].size(); k++) {
                            // skip same point
                            if (results[i][m] == results[j][k])
                                continue;

                            // compute distance diff and angle diff between
                            double score_j;
                            double distDiff = fabs(sqrt(pow(detectPoints[i].x - detectPoints[j].x, 2) +
                                                        pow(detectPoints[i].y - detectPoints[j].y, 2)) -
                                                   sqrt(pow(realPoints[results[i][m]].x - realPoints[results[j][k]].x,
                                                            2) +
                                                        pow(realPoints[results[i][m]].y - realPoints[results[j][k]].y,
                                                            2)));

                            double angleDiff = normalDiff(
                                    atan2(detectPoints[i].y - detectPoints[j].y, detectPoints[i].x - detectPoints[j].x),
                                    atan2(realPoints[results[i][m]].y - realPoints[results[j][k]].y,
                                          realPoints[results[i][m]].x - realPoints[results[j][k]].x));

                            double distscore = scoreFunc(distDiff, distScoreBase_, distRatio_);
                            double anglescore = scoreFunc(angleDiff, angleScoreBase_, angleRatio_);
                            score_j = distscore + anglescore;

#if log_lv1
                            // comupte score
                            printf("score [%d,%d,%d,%d], %f , %f \n", i, m, j, k, distscore, anglescore);
                            std::cout << std::endl;
#endif
                            // update best_score_j
                            if (score_j > best_score_j && score_j > matchScore_) {
                                best_score_j = score_j;
                            }


                        }
                        //end loop

                        // update
                        if (best_score_j > matchScore_) {
                            score_i += best_score_j;
                            match_j++;
                        }


                    }
                }
                if (match_j > 0) {
                    score_i /= match_j;
                    // end loop m
                    // update best_score_i

                    if (score_i > best_score_i) {
                        best_score_i = score_i;
                        best_id_i = m;
                    }
                }


            }
            // ==== ok
            // end loop i : ok
            // update assignment
            if (best_score_i > finalScore_) {
                // fix remove invalid assign
                if (results[i][best_id_i] > realPoints.size() - 1) {
                    results[i].clear();
                }

                tuple<int, int> point_assign = std::make_tuple(i, results[i][best_id_i]);
#if log_lv1

                printf("pair [%d,%d]\n", i, results[i][best_id_i]);
                std::cout << std::endl;
#endif
                assignments.push_back(point_assign);
            } else {
                // clear
                results[i].clear();
            }


//            assignments.push_back(std::make_tuple(i,i));
        }


    }

    // fix bug
    // check if assignments is stric one to one
    // main check if more than one detect assign with a same realboard
    map<int, int> mapcheck;
    vector<int> remove_ids;
    for (int i = 0; i < assignments.size(); i++) {
        int ass_1 = std::get<1>(assignments[i]);
        if (mapcheck[ass_1] == 0) {
            mapcheck[ass_1] = 1;
        } else {
            mapcheck[ass_1]++;
        }

        // check
        if (mapcheck[ass_1] > 1) {
            // remove the small length one
            vector<int> check_ids;
            for (int j = 0; j < assignments.size(); j++) {
                int check_id = std::get<1>(assignments[j]);
                if (check_id == ass_1) {
                    check_ids.push_back(std::get<0>(assignments[j]));
                }
            }

            // choose id to remove
            // choose the longger board
            int best_id = 0;
            double best_length = 0;
            for (int k = 0; k < check_ids.size(); k++) {
                if (detectPoints[check_ids[k]].length > best_length) {
                    best_id = k;
                    best_length = detectPoints[check_ids[k]].length;
                }
            }

            // add ohter id to remove ids
            for (int k = 0; k < check_ids.size(); k++) {
                if (k != best_id) {
                    remove_ids.push_back(check_ids[k]);
                }
            }
        }
    }
    auto new_assignments = assignments;

    new_assignments.clear();
    for (int i = 0; i < assignments.size(); i++) {
        bool rm = false;
        for (int j = 0; j < remove_ids.size(); j++) {
            if (std::get<0>(assignments[i]) == remove_ids[j]) {
                rm = true;
            }
        }
        if (!rm) {
            new_assignments.push_back(assignments[i]);
        }
    }

    return new_assignments;
}

bool
BoardFinder::findNN(vector<Position> &realPointsW, vector<Position> &realPoints, vector<Position> &detectPoints) {

    // first find nearest points within radius
    // for those can't tell aprt from kdtree search, use pattern match



    // if point Num >=2
    // use pattern matncher
    // if only detect one ;
    // use kdtree search

    vector<Position> realPointReg, detecctPointReg;
    vector<tuple<int, int> > assign;


    // search with kd tree search
    // remove points which is well matched; which is one to one match

    // match reamined points
#if 1
    assign = kdTreeMatch(detectPoints, realPoints);
    if (assign.empty() || (!detectOne_ && assign.size() == 1)) {

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

#if 1
    for (int i = 0; i < realPointsW.size(); i++) {
        printf("realw--real--detect = \n[%.3f,%.3f] <-[%.3f,%.3f]-> [%.3f,%.3f]", realPointsW[i].x, realPointsW[i].y,
               realPoints[std::get<1>(assign[i])].x, realPoints[std::get<1>(assign[i])].y, detectPoints[i].x,
               detectPoints[i].y);

        std::cout << std::endl;
    }
#endif

    return true;
#endif
#if 0
    // how to sort with assigment
    // select fisr match point as start angle , sort
    if (!assign.empty() || (assign.size() ==1 && detectOne_)){

    }

    // send last assignment to match function
    assign = patternMatcher.match(detectPoints, realPoints);




    if (detectPoints.size() > 1) {
        // call pattern match
        // how to detect failure
        assign = patternMatcher.match(detectPoints, realPoints);
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

        if (!detectOne_){
            return false;
        }




        realPointsW = realPointReg;
        detectPoints = detecctPointReg;
        return true;

        // check results, remove duplicate or invalid points;


        // start with a point matcn only one point
        // maybe it's fake detection


        // clear and check distancce  and sort vector

    }
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
    tf::StampedTransform tmp_mapOdomTf_ = tf::StampedTransform(mapTOodomTf,
                                                               transform_expiration,
                                                               fixed_frame_id_, odom_frame_id_);
    tf::Transform tmpamcl_mapOdomTf_;
    tf::poseMsgToTF((*mapOdom_data_).pose.pose, tmpamcl_mapOdomTf_);
    double change_length = (tmp_mapOdomTf_ * mapOdomTf_.inverse()).getOrigin().length();
    double change_angle = tf::getYaw((tmp_mapOdomTf_ * mapOdomTf_.inverse()).getRotation());
#if 1
    if (lastPublishOk_ && (change_length < static_dist_ && fabs(change_angle) < static_angle_)) {

//        mapOdomTf_ = tmp_mapOdomTf_;
        return;

    }
#endif

    // fix , avoid big change
#if 1
    change_length = (tmpamcl_mapOdomTf_ * mapOdomTf_.inverse()).getOrigin().length();
    change_angle = tf::getYaw((tmpamcl_mapOdomTf_ * mapOdomTf_.inverse()).getRotation());
    if (lastPublishOk_ &&
        (change_length > amcl_diff_max_ * static_dist_ || fabs(change_angle) > amcl_diff_max_ * static_angle_)) {

//        mapOdomTf_ = tmp_mapOdomTf_;
        return;

    }
#endif


    mapOdomTf_ = tmp_mapOdomTf_;



    geometry_msgs::Pose debug_odom;
    tf::poseTFToMsg(mapOdomTf_, debug_odom);
#if 0
    ROS_ERROR_STREAM("latest updated odom\n"<<debug_odom);
    if (debug_odom.position.x < 17 || debug_odom.position.x >19){
        ROS_ERROR_STREAM("exit at latest updated odom\n"<<debug_odom);

        exit(0);
    }
#endif
#if log_lv1

    ROS_ERROR_STREAM("update odom \n" << debug_odom);
#endif

    // segementation  fault: use shared ptr before initialise
    tf::StampedTransform tmp(mapOdomTf_);
    std::swap(*mapToodomtfPtr_, tmp);

    // if thread is not running ; start running
    if (!threadClass_.isRunning()) {
#if 1
        // todo ; pypass thread
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

        //fix

        if (fabs(realPoints[0].y - realPoints[size - 1].y) < 0.1 ||
            fabs(detectPoints[0].y - detectPoints[size - 1].y) < 0.1)
            return;
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
    translation.z = baseLaserTf_.getOrigin().z();

    realTarget = tf_util::createTransformFromTranslationYaw(translation, realyaw);
    translation.x = detectX;
    translation.y = detectY;
    translation.z = baseLaserTf_.getOrigin().z();
    detectTarget = tf_util::createTransformFromTranslationYaw(translation, detectyaw);


    //compute laser pose

    laserPose = realTarget * detectTarget.inverse();

    geometry_msgs::Pose debug_pose;
    tf::poseTFToMsg(laserPose, debug_pose);
#if log_lv1

    ROS_ERROR_STREAM("laser pose \n" << debug_pose);
#endif




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
    auto x_cov = param_["x_cov"].As<double>(0.05);
    auto y_cov = param_["y_cov"].As<double>(0.05);
    auto yaw_cov = param_["yaw_cov"].As<double>(0.03);


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

#if log_lv1

    ROS_INFO("set filter cnt: %d", int(srv.request.pose_array_msg.poses.size()));
#endif

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
        bool match = findNN(realPointsW, realPoints, detectPoints);

        // if detect more board
        // 1 board: compute with position and yaw
        // 2 and more , compute with relative position of betwwen board

        if (match) {
            cout << endl << "match ok!" << endl;
            // compute target vector
            computeUpdatedPose(realPointsW, detectPoints);
#if 1
            int resetDuration_ = param_["resetDuration"].As<int>(3);
            matchNum_++;

            if (matchNum_ % resetDuration_ == 0) {
                resetAmcl();

                if (matchNum_ > 0) {
                    matchNum_ = 0;
                }
            }

#endif
            lastPublishOk_ = true;

        } else {
            cout << endl << "match failed! no match board" << endl;

            // find no match board
            tf::Transform transform;
            tf::poseMsgToTF(laserPose_, transform);
            updateMapOdomTf(transform);
            lastPublishOk_ = false;
            matchNum_ = 1;
        }


    } else {
        cout << endl << "match failed! detect no board" << endl;

        // detect no board
        tf::Transform transform;
        tf::poseMsgToTF(laserPose_, transform);
        updateMapOdomTf(transform);
        lastPublishOk_ = false;
        matchNum_ = 1;


    }
}



