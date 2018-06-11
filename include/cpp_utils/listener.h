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

#include <vector>
#include <string>
#include <map>
#include <tuple>

using std::vector;
using std::string;
using std::map;
using std::tuple;

namespace util {


    class Listener {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        //call callback_queue given specific topic
        map<string, boost::shared_ptr<ros::CallbackQueue>> callbackqueue_;

        tf::TransformListener *tf_;

        // check if get update message
        bool updated_;

        // check if topic callback queue exists in map
        bool topicExists(string topic);


    public:
        Listener(ros::NodeHandle nh, ros::NodeHandle nh_private);

        Listener();

        ~Listener();

        /*create normal subscriber, given topic name and buffer length
         * return tuple < shared_ptr, subscriber>
         * */
        template<class T>
        tuple<boost::shared_ptr<T>, ros::Subscriber> createSubcriber(string topic, unsigned int buffer_size);

//
        template<class T>
        tuple<boost::shared_ptr<T>, boost::shared_ptr<tf::MessageFilter<T>>, boost::shared_ptr<message_filters::Subscriber<T>>>
        createSubcriberFilteredTf(string topic, unsigned int buffer_size, string frame);

        template<class T>
        std::shared_ptr<T> getChat(string topic);

        template<class T>
        void callback(const typename T::ConstPtr &msg);

        template<class T>
        void bindcallback(const typename T::ConstPtr &msg, boost::shared_ptr<T> &data);

        bool getOneMessage(string topic, double wait);


    };

    Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nh_private) {
        updated_ = false;
        tf_ = new tf::TransformListener();

    }

    Listener::Listener() {
        updated_ = false;
        tf_ = new tf::TransformListener();

    }


    Listener::~Listener() {
        delete tf_;
        tf_ = new tf::TransformListener();

    }

    bool Listener::topicExists(string topic) {
        if (util::keyExists<string, boost::shared_ptr<ros::CallbackQueue>>(callbackqueue_, topic)) {
            return true;
        } else
            return false;
    }


    template<class T>
    tuple<boost::shared_ptr<T>, ros::Subscriber> Listener::createSubcriber(string topic, unsigned int buffer_size) {
//    callbackqueue_.a
        boost::shared_ptr<T> data_ptr(boost::make_shared<T>());
        tuple<boost::shared_ptr<T>, ros::Subscriber> res;

        if (topicExists(topic)) {
            ROS_ERROR("%s topic exists! return empty shared_ptr", topic.c_str());
            return res;

        }

//    boost::shared_ptr<T> data_ptr = boost::shared_ptr<T>(new T());

        // use make_shared whenever you can (i.e. when you don't need a custom deleter

        // create new nodehandler
        ros::NodeHandle n;
        boost::shared_ptr<ros::CallbackQueue> q(boost::make_shared<ros::CallbackQueue>());
        n.setCallbackQueue(q.get());


        ros::Subscriber chat_func_sub = n.subscribe<T>(topic, buffer_size,
                                                       boost::bind(&Listener::bindcallback<T>, this, _1, data_ptr));



//    ros::Subscriber chat_func_sub = nh_.subscribe(topic, 2, &Listener::callback<T>, this);

        callbackqueue_[topic] = q;
        res = std::make_tuple(data_ptr, chat_func_sub);


        return res;

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
    void Listener::bindcallback(const typename T::ConstPtr &msg, boost::shared_ptr<T> &data) {


        data->cmd.data = "233" + msg->cmd.data;

        ROS_INFO_STREAM(*msg);

        updated_ = true;

    }

    bool Listener::getOneMessage(string topic, double wait) {
        updated_ = false;
        if (!topicExists(topic)) {
            ROS_ERROR("%s topic not exists! ", topic.c_str());
            return false;

        }
        if (wait > 0) {
            callbackqueue_[topic].get()->callAvailable(ros::WallDuration(wait));
            if (!updated_)
                return false;

        }
        if (wait < 0 && !updated_) {
            callbackqueue_[topic].get()->callAvailable(ros::WallDuration(0.01));
            ros::Rate(100);
        }
        return true;
    }


    template<class T>
    tuple<boost::shared_ptr<T>, boost::shared_ptr<tf::MessageFilter<T>>, boost::shared_ptr<message_filters::Subscriber<T>>>
    Listener::createSubcriberFilteredTf(string topic, unsigned int buffer_size, string target_frame) {

        //    callbackqueue_.a
        boost::shared_ptr<T> data_ptr(boost::make_shared<T>());
        tuple<boost::shared_ptr<T>, boost::shared_ptr<tf::MessageFilter<T>>, boost::shared_ptr<message_filters::Subscriber<T>>> res;


        if (topicExists(topic)) {
            ROS_INFO("%s topic exists! return empty shared_ptr", topic.c_str());
            return res;

        }

        // create new nodehandler
        ros::NodeHandle n;
        boost::shared_ptr<ros::CallbackQueue> q(boost::make_shared<ros::CallbackQueue>());
        n.setCallbackQueue(q.get());


        //nomal filter with private varibel
#if 0
        topic_sub_ = new message_filters::Subscriber<T>(n, topic, 1);

        //    message_filters::Subscriber<T>t (n, topic, 1);
        //    message_filters::Subscriber<T> * topic_sub =  &(t);
        topic_filter_ =  new tf::MessageFilter<T>(*topic_sub_,*tf_,target_frame,5);
        topic_filter_->registerCallback(boost::bind(&Listener::bindcallback<T>,this, _1, data_ptr));
#endif

        // with shared_ptr
#if 1
        boost::shared_ptr<message_filters::Subscriber<T>> topic_sub(
                boost::make_shared<message_filters::Subscriber<T>>(n, topic, 1));
        boost::shared_ptr<tf::MessageFilter<T>> topic_filter(
                boost::make_shared<tf::MessageFilter<T>>(*topic_sub.get(), *tf_, target_frame, 5));
        topic_filter.get()->registerCallback(boost::bind(&Listener::bindcallback<T>, this, _1, data_ptr));
#endif

        callbackqueue_[topic] = q;
        res = std::make_tuple(data_ptr, topic_filter, topic_sub);
        return res;


    }


}
#endif //CATKIN_STARTUP_LISTENER_H
