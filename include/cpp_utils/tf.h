//
// Created by waxz on 18-6-8.
//

#ifndef CATKIN_STARTUP_TF_H
#define CATKIN_STARTUP_TF_H

#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
using std::vector;
using std::string;
namespace util{


    // TF_NAN_INPUT usually is caused by invalid quaternions being sent. You will need to debug the program sending the quaternions.
    // The Euclidean magnitude of a quaternion should be one. If numerical errors cause a quaternion magnitude other than one,
    // ROS will print warnings. To avoid these warnings, normalize the quaternion:
    tf::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw){

        tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
        q.normalize();


        ROS_INFO_STREAM(q);  // Print the quaternion components (0,0,0,1)
        return q;

    }

    void getRPYFromQuaternion(double &roll, double &pitch, double &yaw, tf::Quaternion q){
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

    void getYawFromQuaternion(double &yaw, tf::Quaternion q){
        yaw = tf::getYaw(q);
    }

    tf::Quaternion createQuaternionFromYaw(double yaw){
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        q.normalize();

        ROS_INFO_STREAM(q);  // Print the quaternion components (0,0,0,1)
        return q;
    }

    tf::Vector3 createVector3FromXYZ(double x, double y, double z){
        return tf::Vector3(x,y,z);
    }

    tf::Vector3 createVector3FromTranslation(geometry_msgs::Point translation){
        return tf::Vector3(translation.x, translation.y, translation.z);
    }


    tf::Transform createTransformFromTranslationYaw(geometry_msgs::Point translation, double yaw){
        tf::Transform transform = tf::Transform(createQuaternionFromYaw(yaw), createVector3FromTranslation(translation));
        return transform;
    }








// lookup

    bool lookupTransform(tf::TransformListener *tf_, string fix_frame, string target_frame, tf::Transform &transform,
                         ros::Time time, double sleep_duration = 0.1, bool block = false) {
        tf::StampedTransform transform_stamped;


        while (ros::ok()){
            try {

                tf_->waitForTransform(fix_frame, target_frame, time, ros::Duration(0.1));
                tf_->lookupTransform(fix_frame, target_frame, time, transform_stamped);
                break;
            }
            catch (tf::TransformException &ex) {
                ROS_WARN("lookup transformation %s to %s failure: \n %s", fix_frame.c_str(), target_frame.c_str(), ex.what());
                ros::Rate(10).sleep();
                if (!block)
                    return false;
            }
        }

        // normalize
        tf::Quaternion q(transform_stamped.getRotation());
        q.normalize();
        transform_stamped.setRotation(q);

        transform = tf::Transform(transform_stamped);
        return true;
    }

    // look up target_frame chane in the fix_frame
    bool
    lookupTranformChange(tf::TransformListener *tf_, ros::Time previous_time, ros::Time current_time, string fix_frame,
                         string target_frame, tf::Transform &transform, double sleep_duration = 0.1,
                         bool block = false) {
        // In case the client sent us a poses in the past, integrate the intervening odometric change.
        tf::StampedTransform transform_stamped;
        while (ros::ok()){
            try
            {
                ros::Time now = ros::Time::now();
                // wait a little for the latest tf to become available

                tf_->waitForTransform(target_frame, previous_time,
                                      target_frame, current_time,
                                      fix_frame, ros::Duration(sleep_duration));
                tf_->lookupTransform(target_frame, previous_time,
                                     target_frame, current_time,
                                     fix_frame, transform_stamped);
                break;
            }
            catch(tf::TransformException e)
            {
                // If we've never sent a transform, then this is normal, because the
                // global_frame_id_ frame doesn't exist.  We only care about in-time
                // transformation for on-the-move pose-setting, so ignoring this
                // startup condition doesn't really cost us anything.
                ROS_WARN("Failed to transform poses in time: (%s)", e.what());
                transform_stamped.setIdentity();
                if (!block)
                    return false;
            }
        }

        // normalize
        tf::Quaternion q(transform_stamped.getRotation());
        q.normalize();
        transform_stamped.setRotation(q);

        transform = tf::Transform(transform_stamped);
        return true;


    }

//Transforms a geometry_msgs PointStamped message to frame target_frame, returns a new PointStamped message.
    geometry_msgs::PointStamped
    transformPoint(tf::TransformListener *tf_, string fix_frame, geometry_msgs::PointStamped point) {
        geometry_msgs::PointStamped new_point;
        tf_->transformPoint(fix_frame, point, new_point);
        return new_point;

    }


    tf::Stamped<tf::Pose> createIdentStampedPose(string frame, ros::Time time = ros::Time::now()) {
        tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                                  tf::Vector3(0, 0, 0)),
                                    time, frame);
        return ident;
    }


    bool getFramePose(tf::TransformListener *tf_, string fix_frame, string target_frame, ros::Time time,
                      tf::Stamped<tf::Pose> &target_pose, double sleep_duration = 0.1, bool block = false) {

        tf::Stamped<tf::Pose> ident = createIdentStampedPose(target_frame, time);


        while (ros::ok()) {
            try {

                tf_->waitForTransform(fix_frame, target_frame, time, ros::Duration(sleep_duration));
                tf_->transformPose(fix_frame, ident, target_pose);
                break;

            }
            catch (tf::TransformException &ex) {
                ROS_WARN("lookup transformation %s to %s failure: \n %s", fix_frame.c_str(), target_frame.c_str(),
                         ex.what());
                if (!block)
                    return false;
            }

        }


        return true;
    }


    // publish
    void sendTranform(tf::TransformBroadcaster *tfb_, tf::Transform transform, string fix_frame, string target_frame,
                      double tolerance = 0.1) {

        ros::Duration transform_tolerance;
        transform_tolerance.fromSec(tolerance);

        ros::Time tn = ros::Time::now();
        ros::Time transform_expiration = (tn +
                                          transform_tolerance);

//            ROS_INFO("update odom by threads");
        tf::StampedTransform transformstamped(transform,
                                              transform_expiration,
                                              fix_frame, target_frame);


        tfb_->sendTransform(transformstamped);
    }

}
#endif //CATKIN_STARTUP_TF_H
