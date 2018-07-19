//
// Created by waxz on 18-6-11.
//

#ifndef CATKIN_STARTUP_LISTENER_H
#define CATKIN_STARTUP_LISTENER_H
//ros
#include <ros/ros.h>
#include <ros/callback_queue.h> //  ros::CallbackQueue queue
#include "tf/message_filter.h"  // filter message with tf
#include <message_filters/subscriber.h>


//util
#include <cpp_utils/container.h>
#include <cpp_utils/tf.h>

#include <vector>
#include <string>
#include <map>
#include <tuple>

using std::vector;
using std::string;
using std::map;
using std::tuple;

namespace rosnode {


    class Listener {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        //call callback_queue given specific topic
        map<string, std::shared_ptr<ros::CallbackQueue>> callbackqueue_;

        map<string, std::shared_ptr<ros::ServiceClient>> servicequeue_;

        vector<ros::Subscriber> subscriberVec_;

        tf::TransformListener *tf_;
        tf::TransformBroadcaster *tfb_;

        // check if get update message
        bool updated_;

        // check if topic callback queue exists in map
        bool topicExists(string topic);

        // check if service exist
        bool serviceExists(string service_name);

    public:
        Listener(ros::NodeHandle nh, ros::NodeHandle nh_private);


        ~Listener();

        /*create normal subscriber, given topic name and buffer length
         * return tuple < shared_ptr, subscriber>
         * */
        template<class T>
        tuple<std::shared_ptr<T>, ros::Subscriber> createSubcriber(string topic, unsigned int buffer_size);

//
        template<class T>
        tuple<std::shared_ptr<T>, std::shared_ptr<tf::MessageFilter<T>>, std::shared_ptr<message_filters::Subscriber<T>>>
        createSubcriberFilteredTf(string topic, unsigned int buffer_size, string frame);

        template<class T>
        bool createServiceClient(string service_name);

        template<class T>
        bool callService(string service_name, T &srv);

        template<class T>
        std::shared_ptr<T> getChat(string topic);

        template<class T>
        void callback(const typename T::ConstPtr &msg);

        template<class T>
        void bindcallback(const typename T::ConstPtr &msg, std::shared_ptr<T> data);

        virtual void doSomething() {}


        bool getOneMessage(string topic, double wait = false);

        bool getTransform(string fix_frame, string target_frame, tf::Transform &transform,
                          ros::Time time = ros::Time::now(), double sleep_duration = 0.1, bool block = false);

        void sendTransform(string fix_frame, string target_frame, tf::Transform &transform, double tolerance = 0.1);

        template<class T>
        bool createThred();

        bool tranformPoints(vector<geometry_msgs::PointStamped> &Points, string fix_frame);

        bool tranformPose(geometry_msgs::PoseStamped &pose, string fix_frame);
    };

// *****
    inline    Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nh_private) {
        updated_ = false;
        tf_ = new tf::TransformListener();
        tfb_ = new tf::TransformBroadcaster();

    }


    inline    Listener::~Listener() {
        delete tf_;
        delete tfb_;

    }

    inline bool Listener::topicExists(string topic) {
        if (container::keyExists<string, std::shared_ptr<ros::CallbackQueue>>(callbackqueue_, topic)) {
            return true;
        } else
            return false;
    }

    inline bool Listener::serviceExists(string service_name) {
        //map<string, std::shared_ptr<ros::ServiceClient>>
        if (container::keyExists<string, std::shared_ptr<ros::ServiceClient>>(servicequeue_, service_name)) {
            return true;
        } else
            return false;
    }

    //serviceExists


    template<class T>
    tuple<std::shared_ptr<T>, ros::Subscriber> Listener::createSubcriber(string topic, unsigned int buffer_size) {
//    callbackqueue_.a
        std::shared_ptr<T> data_ptr(std::make_shared<T>());
        tuple<std::shared_ptr<T>, ros::Subscriber> res;

        if (topicExists(topic)) {
            ROS_ERROR("%s topic exists! return empty shared_ptr", topic.c_str());
            return res;

        }

//    std::shared_ptr<T> data_ptr = std::shared_ptr<T>(new T());

        // use make_shared whenever you can (i.e. when you don't need a custom deleter

        // create new nodehandler
        ros::NodeHandle n;
        std::shared_ptr<ros::CallbackQueue> q(std::make_shared<ros::CallbackQueue>());
        n.setCallbackQueue(q.get());


        ros::Subscriber chat_func_sub = n.subscribe<T>(topic, buffer_size,
                                                       boost::bind(&Listener::bindcallback<T>, this, _1, data_ptr));



//    ros::Subscriber chat_func_sub = nh_.subscribe(topic, 2, &Listener::callback<T>, this);

        callbackqueue_[topic] = q;
        res = std::make_tuple(data_ptr, chat_func_sub);
        subscriberVec_.push_back(chat_func_sub);


        return res;

    }

    template<class T>
    bool Listener::createServiceClient(string service_name) {
        tuple<std::shared_ptr<T>, ros::Subscriber> res;
        // check if service exist
        if (serviceExists(service_name)) {
            ROS_ERROR("%s service_name exists! return empty shared_ptr", service_name.c_str());
            return false;

        }

        // create a serviceclient
        ros::ServiceClient client;

        client = nh_.serviceClient<T>(service_name);
        std::shared_ptr<ros::ServiceClient> s = std::make_shared<ros::ServiceClient>(client);

        servicequeue_[service_name] = s;

        return true;
    }

    template<class T>
    bool Listener::callService(string service_name, T &srv) {
        // check if service exist
        if (!serviceExists(service_name)) {
            ROS_ERROR("%s service_name not exists! ", service_name.c_str());
            return false;

        }

        //call service

        if (servicequeue_[service_name].get()->call(srv)) {
            ROS_INFO("node call service %s ok!", service_name.c_str());
            return true;
        } else {
            ROS_ERROR("node Failed to call service %s", service_name.c_str());
            return false;
        }
    }


    template<class T>
    std::shared_ptr<T> Listener::getChat(string topic) {
        T data;
        data.cmd.data = topic;

        std::shared_ptr<T> p1 = std::make_shared<T>(data);
        return p1;

    }

//template namespace
    template<class T>
    void Listener::callback(const typename T::ConstPtr &msg) {
//    data = *msg;

        ROS_INFO("receive msg.simple");

        updated_ = true;

    }

    template<class T>
    void Listener::bindcallback(const typename T::ConstPtr &msg, std::shared_ptr<T> data) {


        // swap memory
        T m = *msg;
        std::swap(*data, m);
        updated_ = true;
//        ROS_ERROR("get data updated_ ");

    }

    inline bool Listener::getOneMessage(string topic, double wait) {
        updated_ = false;

        if (!topicExists(topic)) {
            ROS_ERROR("%s topic not exists! ", topic.c_str());
            return false;

        }
        if (wait > 0) {
            callbackqueue_[topic].get()->callAvailable(ros::WallDuration(wait));
            if (!updated_) {
                ROS_ERROR("%s topic No update ", topic.c_str());

                return false;

            }

        }

        if (wait < 0 && !updated_) {
            while (ros::ok() && !updated_) {
                callbackqueue_[topic].get()->callAvailable(ros::WallDuration(0.1));
                ros::Rate(100);
//                ROS_ERROR("get data");
            }


        }
//        ROS_ERROR("get data done ");

        return true;
    }


    template<class T>
    tuple<std::shared_ptr<T>, std::shared_ptr<tf::MessageFilter<T>>, std::shared_ptr<message_filters::Subscriber<T>>>
    Listener::createSubcriberFilteredTf(string topic, unsigned int buffer_size, string target_frame) {

        //    callbackqueue_.a
        std::shared_ptr<T> data_ptr(std::make_shared<T>());
        tuple<std::shared_ptr<T>, std::shared_ptr<tf::MessageFilter<T>>, std::shared_ptr<message_filters::Subscriber<T>>> res;


        if (topicExists(topic)) {
            ROS_ERROR("%s topic exists! return empty shared_ptr", topic.c_str());
            return res;

        }

        // create new nodehandler
        ros::NodeHandle n;
        std::shared_ptr<ros::CallbackQueue> q(std::make_shared<ros::CallbackQueue>());
        n.setCallbackQueue(q.get());


        //nomal filter with private varibel
#if 0
        /*
    define topic_sub_ topic_filter_ as member varibel
    */
    topic_sub_ = new message_filters::Subscriber<T>(n, topic, 1);
    topic_filter_ =  new tf::MessageFilter<T>(*topic_sub_,*tf_,target_frame,5);
    topic_filter_->registerCallback(boost::bind(&Listener::bindcallback<T>,this, _1, data_ptr));
#endif

        // with shared_ptr
#if 1
        std::shared_ptr<message_filters::Subscriber<T>> topic_sub(
                std::make_shared<message_filters::Subscriber<T>>(n, topic, 1));
        std::shared_ptr<tf::MessageFilter<T>> topic_filter(
                std::make_shared<tf::MessageFilter<T>>(*topic_sub.get(), *tf_, target_frame, 10));
        topic_filter.get()->registerCallback(boost::bind(&Listener::bindcallback<T>, this, _1, data_ptr));
#endif

        callbackqueue_[topic] = q;
        res = std::make_tuple(data_ptr, topic_filter, topic_sub);
        return res;


    }

    inline bool Listener::getTransform(string fix_frame, string target_frame, tf::Transform &transform, ros::Time time,
                                       double sleep_duration, bool block) {
        ROS_INFO("Listener lookup %s to %s ", fix_frame.c_str(), target_frame.c_str());

        bool successful = tf_util::lookupTransform(tf_, fix_frame, target_frame, transform, time, sleep_duration,
                                                   block);


        geometry_msgs::Pose p;
        tf::poseTFToMsg(transform, p);
        return successful;
    }

    inline void
    Listener::sendTransform(string fix_frame, string target_frame, tf::Transform &transform, double tolerance) {
        tf_util::sendTranform(tfb_, transform, fix_frame, target_frame, tolerance);
    }


    //tranformPoint
    inline bool Listener::tranformPoints(vector<geometry_msgs::PointStamped> &points, string fix_frame) {
        return tf_util::transformPoints(tf_, fix_frame, points, 0.1, true);
    }

    //transformpose
    inline bool Listener::tranformPose(geometry_msgs::PoseStamped &pose, string fix_frame) {
        return tf_util::transformPose(tf_, fix_frame, pose, 0.1, true);
    }

}
#endif //CATKIN_STARTUP_LISTENER_H
