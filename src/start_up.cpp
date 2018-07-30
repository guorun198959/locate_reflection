//
// Created by waxz on 18-6-8.
//

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <tuple>
#include <ctime>
#include <valarray>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <cpp_utils/random.h>

#include <cpp_utils/parse.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <cpp_utils/listener.h>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CVUI_IMPLEMENTATION

#include <cpp_utils/cvui.h>

using std::vector;
using std::tuple;
using std::valarray;
using std::map;
#define WINDOW_NAME    "Sparkline"


class Gui {
private:
    string windowName_;
    map<string, bool> boolMap_;
    map<string, float> numMap_;
public:
    Gui(cv::Mat &bg, string windowName);

    cv::Mat bg_;
    cv::Mat bg_copy_;

    void createCheckBox(int x, int y, string content, bool &state);

    void update();

    void show();

    bool getBool(string key);

    float getNum(string key);

};

Gui::Gui(cv::Mat &bg, string windowName) : bg_(bg), windowName_(windowName) {
    bg_copy_ = bg_.clone();
}

void Gui::createCheckBox(int x, int y, string content, bool &state) {
    cvui::checkbox(bg_copy_, x, y, "update data", &state);

}

void Gui::update() {
    // create ui


    // all the behind the scenes magic to handle mouse clicks, etc.
    cvui::update();


}

void Gui::show() {
    cv::imshow(windowName_, bg_copy_);
    bg_copy_ = bg_.clone();

}

int main(int argc, char **argv) {

    cv::Mat frame = cv::Mat(600, 1000, CV_8UC3);


    // Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
    cvui::init(WINDOW_NAME);
    cv::Point anchor;
    cv::Rect roi(0, 0, 0, 0);


    // get odom, vel_mb,base_link




    ros::init(argc, argv, "cvui");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    rosnode::Listener l(nh, nh_private);

    ros::Time tn = ros::Time::now();

    tf::Transform odombase_transform, mapbase_transform;

    geometry_msgs::Pose pose, pose2;

    std::shared_ptr<geometry_msgs::Twist> vel_data;
    auto res = l.createSubcriber<geometry_msgs::Twist>("vel_mb", 5);
    vel_data = std::get<0>(res);


    double rate = 1;
    ros::Rate r(rate);

    valarray<double> yaw1(0.0, 1000), yaw2(0.0, 1000), vels(0.0, 1000), accu_angles(0.0, 1000);

    int cnt = 0;
    bool update = false;
    bool bt = false;
    double accu_angle = 0.0;
    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {
        frame = cv::Scalar(255, 255, 255);
#if 1
        tn = ros::Time::now();
        bool get1 = l.getTransform("odom", "base_link", odombase_transform, tn, 0.01);
        tf::poseTFToMsg(odombase_transform, pose);
        bool get2 = l.getTransform("map", "base_link", mapbase_transform, tn, 0.01);
        tf::poseTFToMsg(mapbase_transform, pose2);

        bool get3 = l.getOneMessage("vel_mb", 0.05);

        update = (get1 && get2 && get3);
#endif
        cvui::checkbox(frame, 0, 40, "update data", &bt);





        // render box
        if (cvui::mouse(cvui::DOWN)) {
            // Position the anchor at the mouse pointer.
            anchor.x = cvui::mouse().x;
            anchor.y = cvui::mouse().y;

            // Inform we are working, so the ROI window is not updated every frame
        }
        // Is any mouse button down (pressed)?
        if (cvui::mouse(cvui::IS_DOWN)) {
            // Adjust roi dimensions according to mouse pointer
            int width = cvui::mouse().x - anchor.x;
            int height = frame.rows;

            roi.x = width < 0 ? anchor.x + width : anchor.x;
            roi.y = 0;
            roi.width = std::abs(width);
            roi.height = std::abs(height);

            // Show the roi coordinates and size
            int s = roi.x;
            int e = roi.x + roi.width;
            valarray<double> tmp = vels[std::slice(s, roi.width, 1)];
            cvui::printf(frame, 0, 20, 0.3, 0xff0000,
                         "in this time duration , data change : yaw1 = %f, yaw2 = %f, vel = %f , accu_angle = %f ",
                         yaw1[e] - yaw1[s], yaw2[e] - yaw2[s], vels[e] - vels[s], accu_angles[e] - accu_angles[s]);


        }


        // warn: roi may be out of window
        // must check bounding
        // Ensure ROI is within bounds
        roi.x = roi.x < 0 ? 0 : roi.x;
        roi.y = roi.y < 0 ? 0 : roi.y;
        roi.width = roi.x + roi.width > frame.cols ? roi.width + frame.cols - (roi.x + roi.width) : roi.width;
        roi.height = roi.y + roi.height > frame.rows ? roi.height + frame.rows - (roi.y + roi.height) : roi.height;
        cvui::rect(frame, roi.x, roi.y, roi.width, roi.height, 0xff0000);
#if 1
        int s = cvui::mouse().x;
        cvui::printf(frame, 0, 40, 0.3, 0xff0000, "at this time: yaw1 = %f, yaw2 = %f , vel = %f ,accu_angle = %f",
                     yaw1[s], yaw2[s], vels[s], accu_angles[s]);

        // update data
        if (update && bt) {
            if (cnt == 999) {
                yaw1 = yaw1.cshift(1);
                yaw2 = yaw2.cshift(1);
                vels = vels.cshift(1);
                accu_angles = accu_angles.cshift(1);
            }
            yaw1[cnt] = tf::getYaw(pose.orientation);
            yaw2[cnt] = tf::getYaw(pose2.orientation);
            vels[cnt] = (*vel_data).angular.z;

            accu_angle += (*vel_data).angular.z * (tn - last_time).toSec();
            accu_angle = atan2(sin(accu_angle), cos(accu_angle));
            accu_angles[cnt] = accu_angle;

            last_time = tn;
            if (cnt < 999) {
                cnt++;
            }
        }
#endif

        std::vector<double> few_points;
        for (std::vector<double>::size_type i = 0; i < 30; i++) {
            few_points.push_back(rand() + 0.);
        }
        // valarray to vector
        vector<double> yaw1vec(&(yaw1[0]), &(yaw1[0]) + 1000);
        vector<double> yaw2vec(&(yaw2[0]), &(yaw2[0]) + 1000);
        vector<double> velvec(&(vels[0]), &(vels[0]) + 1000);
        vector<double> anglevec(&(accu_angles[0]), &(accu_angles[0]) + 1000);

        cvui::sparkline(frame, yaw1vec, 0, 100, 800, 100, 0x00ff00);
        cvui::rect(frame, 0, 100, 800, 100, 0xff0000);

        cvui::sparkline(frame, yaw2vec, 0, 200, 800, 100, 0xff0000);
        cvui::rect(frame, 0, 200, 800, 100, 0xff0000);

        cvui::sparkline(frame, velvec, 0, 300, 800, 100, 0x0000ff);
        cvui::rect(frame, 0, 300, 800, 100, 0xff0000);

        cvui::sparkline(frame, anglevec, 0, 400, 800, 100, 0x0000ff);
        cvui::rect(frame, 0, 400, 800, 100, 0xff0000);

        // all the behind the scenes magic to handle mouse clicks, etc.
        cvui::update();

        // Show everything on the screen
        cv::imshow(WINDOW_NAME, frame);

        // Check if ESC key was pressed
        if (cv::waitKey(20) == 27) {
            break;
        }



#if 0
        r.sleep();
#endif
    }



}