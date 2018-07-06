//
// Created by waxz on 18-7-4.
//

#include "locate_reflection/boardFinder.h"
#include <locate_reflection/patternMatcher.h>


#define debug_pub true

BoardFinder::BoardFinder(ros::NodeHandle nh, ros::NodeHandle nh_private) :
        nh_(nh), nh_private_(nh_private), l(nh, nh_private) {
//    util::Listener l(nh, nh_private);
#if 0
    auto res = l.createSubcriberFilteredTf<sensor_msgs::LaserScan>("scan", 2, "map");

#endif
#if 1

    auto res = l.createSubcriber<sensor_msgs::LaserScan>("scan", 10);
#endif


    laser_data_ = std::get<0>(res);


    auto res2 = l.createSubcriber<geometry_msgs::Pose>("amcl_tf", 1);

    mapOdom_data_ = std::get<0>(res2);

    // read xml
    string filename = "board.yaml";
    try {
        param_ = Yaml::readFile(filename);

    } catch (...) {
        printf("read %s failed!!\n", filename.c_str());
        exit(0);
    }


    baseLaserTf_.setIdentity();
    mapOdomTf_.setIdentity();

//    // todo: debug ,preset odom
//    mapOdomTf_.setOrigin(tf::Vector3(-16.6,10,0.0));

    boardPub = nh_.advertise<geometry_msgs::PoseArray>("detectboard", 2);
    pointPub = nh_.advertise<geometry_msgs::PoseArray>("lightpoint", 2);
    markerPub = nh_.advertise<geometry_msgs::PoseArray>("board", 2);

    patternMatcher = PatternMatcher();

//
//    threading_ = threading_util::Threading<threading_util::ThreadTfPub<tf::StampedTransform>>();
//    tf::TransformBroadcaster * tfb  = new tf::TransformBroadcaster();
//    tf::StampedTransform transformstamped;
//    std::shared_ptr<tf::StampedTransform> data = std::make_shared<tf::StampedTransform>(transformstamped);

//    threading_util::ThreadTfPub<tf::StampedTransform> threadTfPub_(10,data,tfb);
//    threading_.createThread(threadTfPub_);




}

bool BoardFinder::updateSensor() {
    bool getmsg = l.getOneMessage("scan", 0.1);
    size_t size = laser_data_.get()->ranges.size();
    if (bear_.size() != size) {
        bear_ = valarray<float>(0.0, size);
        for (int i = 0; i < size; i++) {
            bear_[i] = laser_data_.get()->angle_min + i * laser_data_.get()->angle_increment;
        }
    }
    return getmsg;
}

bool BoardFinder::getMapOdomTf() {
    bool getmsg = l.getOneMessage("maptoodomtf", 0.1);
    if (getmsg)
        tf::poseMsgToTF(*mapOdom_data_, mapOdomTf_);


    return getmsg;


}

void BoardFinder::getLaserPose() {

    ROS_INFO("getLaserPose start tf");
    getMapOdomTf();
//    l.getOneMessage("scan",-1);
    tf::Transform transform;

    transform.setIdentity();

    l.getTransform("odom", "laser", transform);
    tf::poseTFToMsg(mapOdomTf_ * transform, laserPose_);
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
            tf::poseTFToMsg(tf_util::createTransformFromTranslationYaw(point, p.yaw), pose);
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
                valarray<size_t> idx_val = container::createValarrayFromVector<size_t>(idx_vec);

                valarray<float> cluster_x = lightXs[idx_val];

                double center_x = cluster_x.sum() / cluster_x.size();

                valarray<float> cluster_y = lightYs[idx_val];

                double center_y = cluster_y.sum() / cluster_y.size();
                p.x = center_x;
                p.y = center_y;

                ps.push_back(p);
                cout << "x" << p.x << "y" << p.y << "length:" << length << std::endl;


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

void BoardFinder::transformPoints(vector<Position> &detectPoints, tf::Transform transform) {
    geometry_msgs::Point point;
    for (int i = 0; i < detectPoints.size(); i++) {

        tf::Point p(detectPoints[i].x, detectPoints[i].y, 0.0);
        tf::pointTFToMsg((mapOdomTf_ * transform * baseLaserTf_).inverse() * p, point);

        detectPoints[i].y = point.y;
        detectPoints[i].x = point.x;
    }
}


bool
BoardFinder::findNN(vector<Position> &realPointsW, vector<Position> &realPoints, vector<Position> &detectPoints) {


    vector<Position> realPointReg, detecctPointReg;

    // call pattern match
    // how to detect failure
    auto assign = patternMatcher.match(detectPoints, realPoints);
    if (assign.empty()) {

        cout << "match failure!!" << endl;
        return false;

    }

    for (int i = 0; i < assign.size(); i++) {
        realPointReg.push_back(realPointsW[std::get<1>(assign[i])]);
        detecctPointReg.push_back(detectPoints[std::get<0>(assign[i])]);
    }
    realPointsW = realPointReg;
    detectPoints = detecctPointReg;

    return true;

#if 0
    realPointReg = realPoints;
    bool zerofisrt = false;
    if (realPoints[0].y > realPoints[1].y) {
        zerofisrt = true;
    }
    Position tmp;

    if ((detectPointsW[0].y > detectPointsW[1].y) != (realPoints[0].y > realPoints[1].y)) {
        realPoints[0] = realPointReg[1];
        realPoints[1] = realPointReg[0];

    }

    return;

#endif

#if 0

    // create kdtree
    const int npoints = realPoints.size();
    std::vector<kdtree::Point2d> points(npoints);
    for (int i = 0; i < npoints; i++) {
        points[0] = kdtree::Point2d(realPoints[i].x, realPoints[i].y);
    }


    // build k-d tree
    kdtree::KdTree<kdtree::Point2d> kdtree(points);

    double radius = 2.0;
    // query point
    vector<vector<int> > results;
    for (int i = 0; i < detectPointsW.size(); i++) {
        // create query point
        //search
        vector<int> res;


        kdtree::Point2d query(detectPointsW[i].x, detectPointsW[i].y);
        res = kdtree.queryIndex(query, kdtree::SearchMode::radius, radius);

        results.push_back(res);
    }

    int startidx = 0;
    for (int i = 0; i < results.size(); i++) {
        if (results[i].size() == 1) {

            detecctPointReg.push_back(detectPoints[i]);
            realPointReg.push_back(realPoints[results[i][0]]);
        }
    }

    realPoints = realPointReg;
    detectPoints = detecctPointReg;

    // check results, remove duplicate or invalid points;


    // start with a point matcn only one point
    // maybe it's fake detection


    // clear and check distancce  and sort vector
#endif
}

void BoardFinder::updateMapOdomTf(tf::Transform laserPose, ros::Time time) {

    tf::Transform odomTobase;

    if (!l.getTransform("odom", "base_link", odomTobase, time))
        return;

    if (baseLaserTf_.getOrigin().getX() == 0.0)
        l.getTransform("base_link", "laser", baseLaserTf_);

    tf::Transform mapTOodomTf = laserPose * baseLaserTf_.inverse() * odomTobase.inverse();

    l.sendTransform("map", "odom", mapTOodomTf, 0.1);


}

void BoardFinder::computeUpdatedPose(vector<Position> realPoints, vector<Position> detectPoints) {

    // get points pairs
    // compute target vector
    int size = realPoints.size();
    tf::Transform realTarget, detectTarget, laserPose;
    geometry_msgs::Point translation;
    double realX = 0, realY = 0, detectX = 0, detectY = 0;
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

    translation.x = realX;
    translation.y = realY;
    double realyaw = std::atan2(realPoints[0].y - realPoints[size - 1].y, realPoints[0].x - realPoints[size - 1].x);
    double detectyaw = std::atan2(detectPoints[0].y - detectPoints[size - 1].y,
                                  detectPoints[0].x - detectPoints[size - 1].x);

    realTarget = tf_util::createTransformFromTranslationYaw(translation, realyaw);
    translation.x = detectX;
    translation.y = detectY;
    detectTarget = tf_util::createTransformFromTranslationYaw(translation, detectyaw);

    //compute laser pose

    laserPose = realTarget * detectTarget.inverse();





    // compute base_link pose


    // get odom-baselink

    updateMapOdomTf(laserPose, laser_data_.get()->header.stamp);


    // compute map-odom


}


// find reflection board and update map-odom tf
// get board in map ; given baselink
// get board from scan
// match two point pattern
// compute relative pose
// update tf
void BoardFinder::findLocation() {
    // get real board position
    vector<Position> realPointsW = getBoardPosition();
    vector<Position> realPoints = realPointsW;
    // detect board position
    vector<Position> detectPoints = detectBoard();
//    vector<Position> detectPointsW = detectPoints;

    // get odom to laser transform
    tf::Transform odomTobaseTf;
    if (!l.getTransform("odom", "base_link", odomTobaseTf, laser_data_.get()->header.stamp, 0.1, false)) {
        ROS_ERROR("Can't get odom to laser tf");
//        return;
    }


    // todo: test matcher
    vector<Position> p1, p2;
    p1.push_back(Position(-1, 2, 3.3));
    p1.push_back(Position(1, 2, 3.3));
    p1.push_back(Position(1.5, 1.5, 3.3));

    p2.push_back(Position(-3.1, 1.9, 3.3));
    p2.push_back(Position(-1.1, 1.9, 3.3));
    p2.push_back(Position(0.9, 1.9, 3.3));
    p2.push_back(Position(1.4, 1.4, 3.3));
    realPoints = p2;
    realPointsW = p2;
    detectPoints = p1;




    // if successful
    if (realPoints.size() > 1 && detectPoints.size() > 1) {

        // transform detectPoints to map frame

//        transformPointsToWorld(detectPointsW, mapToLaserTf);




        // find match and sort vector
        // convert map board to laser frame

        //todo:bypass
//        transformPoints(realPoints,odomTobaseTf);
        findNN(realPointsW, realPoints, detectPoints);

        // more than two points
        if (detectPoints.size() > 1) {
            // compute target vector
            computeUpdatedPose(realPointsW, detectPoints);

        }


#if 0
        computeTagetVector();
        getBasePose();
        updateOdom();
#endif

    }
}



