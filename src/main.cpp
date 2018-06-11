#include <iostream>
#include <chrono>
#include <boost/bind.hpp>
// node msg and srv
#include <catkin_startup/mytopic.h>
#include <catkin_startup/myservice.h>
#include <cpp_utils/container.h>

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
using std::vector;
using std::string;
using std::map;
using std::tuple;
namespace node=catkin_startup;


class Listener{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    vector<ros::Subscriber> subscribers_;
    map<string, boost::shared_ptr<ros::CallbackQueue>> callbackqueue_;
    tf::TransformListener *tf_;
    bool updated_;
    ros::CallbackQueue queue;

    bool topicExists(string topic);

public:
    Listener(ros::NodeHandle nh,ros::NodeHandle nh_private);
    Listener();

    ~Listener();

    template <class T>
    boost::shared_ptr<T> createSubcriber(string topic, unsigned int buffer_size);
//
    template <class T>
    boost::shared_ptr<T> createSubcriberFilteredTf(string topic, unsigned int buffer_size, string frame);

    template <class T>
    std::shared_ptr<T>  getChat(string topic);

    template <class T>
    void callback(const typename T::ConstPtr &msg);

    template <class T>
    void bindcallback(const typename T::ConstPtr &msg, boost::shared_ptr<T> &data);

    bool getOneMessage(string topic, double wait);

//
//    void callback(const sensor_msgs::LaserScanConstPtr &msg);
//    void callback(const node::mytopicConstPtr &msg);


};

Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nh_private) {
    updated_ = false;
    tf_ = new tf::TransformListener();

}
Listener::Listener() {
    updated_ = false;
    tf_ = new tf::TransformListener();

}


Listener::~Listener(){
    delete tf_;
    tf_ = new tf::TransformListener();

}

bool Listener::topicExists(string topic) {
    if (util::keyExists<string, boost::shared_ptr<ros::CallbackQueue>>(callbackqueue_, topic)){
        return true;
    } else
        return false;
}

template <class T>
boost::shared_ptr<T> Listener::createSubcriber(string topic, unsigned int buffer_size) {
//    callbackqueue_.a
    boost::shared_ptr<T> data_ptr(boost::make_shared<T>());

    if (topicExists(topic)){
        ROS_ERROR("%s topic exists! return empty shared_ptr", topic.c_str());
        return data_ptr;

    }

//    boost::shared_ptr<T> data_ptr = boost::shared_ptr<T>(new T());

    // use make_shared whenever you can (i.e. when you don't need a custom deleter

    // create new nodehandler
    ros::NodeHandle n;
    boost::shared_ptr<ros::CallbackQueue> q(boost::make_shared<ros::CallbackQueue>());
    n.setCallbackQueue(q.get());




    ros::Subscriber chat_func_sub = n.subscribe<T>(topic, buffer_size, boost::bind(&Listener::bindcallback<T>, this,  _1,data_ptr));



//    ros::Subscriber chat_func_sub = nh_.subscribe(topic, 2, &Listener::callback<T>, this);

    subscribers_.push_back(chat_func_sub);
    callbackqueue_[topic] = q;

    return data_ptr;

}

template <class T>
std::shared_ptr<T>  Listener::getChat(string topic){
    T data;
    data.cmd.data = topic;

    std::shared_ptr<T> p1 = std::make_shared<T>(data);
    return  p1;

}

//template namespace
template <class T>
void Listener::callback(const typename T::ConstPtr &msg) {
//    data = *msg;

    ROS_INFO("receive msg.simple");

    updated_ = true;

}

template <class T>
void Listener::bindcallback(const typename T::ConstPtr &msg, boost::shared_ptr<T> &data) {


    data->cmd.data = "233" + msg->cmd.data;

    ROS_INFO_STREAM(*msg);

    updated_ = true;

}

bool Listener::getOneMessage(string topic, double wait) {
    updated_ = false;
    if (!topicExists(topic)){
        ROS_ERROR("%s topic not exists! ", topic.c_str());
        return false;

    }
    if (wait > 0){
        callbackqueue_[topic].get()->callAvailable(ros::WallDuration(wait));
        if (!updated_)
            return false;

    }
    if (wait < 0 && !updated_){
        callbackqueue_[topic].get()->callAvailable(ros::WallDuration(0.01));
        ros::Rate(100);
    }
    return true;
}


template <class T>
boost::shared_ptr<T> Listener::createSubcriberFilteredTf(string topic, unsigned int buffer_size, string target_frame) {

    //    callbackqueue_.a
    boost::shared_ptr<T> data_ptr(boost::make_shared<T>());

    if (topicExists(topic)){
        ROS_INFO("%s topic exists! return empty shared_ptr", topic.c_str());
        return data_ptr;

    }

    // create new nodehandler
    ros::NodeHandle n;
    boost::shared_ptr<ros::CallbackQueue> q(boost::make_shared<ros::CallbackQueue>());
    n.setCallbackQueue(q.get());



//    auto *topic_sub = new message_filters::Subscriber<T>(n, topic, 1);
    boost::shared_ptr<message_filters::Subscriber<T>> sub(boost::make_shared<message_filters::Subscriber<T>>(n, topic, 1));


//    auto *topic_filter_ = new tf::MessageFilter<T>(*sub.get(),*tf_,target_frame,5);
    boost::shared_ptr<tf::MessageFilter<T>> sub_filtered(boost::make_shared<tf::MessageFilter<T>>(*sub.get(),*tf_,target_frame,5));

    sub_filtered.get()->registerCallback(boost::bind(&Listener::bindcallback,this, _1, data_ptr));

//    subscribers_.push_back(topic_filter_);
    callbackqueue_[topic] = q;
    return data_ptr;


}




int main(int argc, char **argv) {

    tuple<int,int,int> t(1,2,3);
    std::cout<<std::get<0>(t);
    return 0;


    std::cout << "Hello, World!" << std::endl;
    //init a node
    ros::init(argc, argv, "start_up");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //**** topic
    // publish on o topic
    ros::Publisher chat_pub = nh.advertise<node::mytopic>("chat",1);

    Listener l;
    std::shared_ptr<node::mytopic> p1 = l.getChat<node::mytopic>("chat");
    if (p1)
        ROS_INFO("%s",p1.get()->cmd.data.c_str());
    node::mytopic msg;
//    boost::shared_ptr<node::mytopic> data = boost::shared_ptr<node::mytopic>(new node::mytopic());
//    data->cmd.data = "sd";
    ros::spinOnce();
    boost::shared_ptr<node::mytopic> data = l.createSubcriber<node::mytopic>("chat",2);

    ros::Rate rate(10);
    int i=0;
    while (ros::ok()){
        i++;
        chat_pub.publish(msg);


        char tmp[200];
        sprintf(tmp,"start %d",i);
        msg.cmd.data =string(tmp) ;
        ros::spinOnce();

        l.getOneMessage("chat",0.1);
        if(data){
            ROS_INFO("local receive %s",data.get()->cmd.data.c_str());
        }


        rate.sleep();
        ros::spinOnce();

    }

    // test code



    return 0;
}