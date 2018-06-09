#include <iostream>
#include <chrono>
#include <boost/bind.hpp>
// node msg and srv
#include <catkin_startup/mytopic.h>
#include <catkin_startup/myservice.h>

#include <ros/ros.h>
#include <ros/callback_queue.h> //  ros::CallbackQueue queue
#include "tf/message_filter.h"  // filter message with tf

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
using std::vector;
using std::string;
using std::map;
namespace node=catkin_startup;

// function callback
void chat_func_cbk(const node::mytopic::ConstPtr &msg){
    std::string log;
    std::string func_name = __FUNCTION__;
    func_name = ", called function: " + func_name;
    log = "get msg: " + msg->cmd.data + func_name;
    ROS_INFO("%s ",log.c_str());
}
// class function callback
class Chatter{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    //**** ros node member
    // topic pub and sub
    ros::Subscriber chat_sub_;
    ros::Publisher chat_pub_;
    // filter

    // service
    ros::ServiceServer chat_service_;
    ros::ServiceClient chat_service__client_;
    //Persistent Connections client
    ros::ServiceClient persistent_client_;


    // call back queue
    ros::CallbackQueue queue;





    //**** method
    void init_params();

public:
    Chatter(ros::NodeHandle nh, ros::NodeHandle nh_private):nh_(nh), nh_private_(nh_private){

//        nh_.setCallbackQueue(&queue);

        // topic
        chat_sub_ = nh_.subscribe("chat",2,&Chatter::chat_func_cbk,this);
        chat_pub_ = nh_.advertise<node::mytopic>("chat2",2);

        // service
        chat_service_ = nh_.advertiseService("chat_service", &Chatter::chat_service_cbk, this);

        chat_service__client_ = nh_.serviceClient<node::myservice>("chat_service");
        //Persistent Connections client
        persistent_client_ = nh_.serviceClient<node::myservice>("chat_service", true);





    }
    void chat_func_cbk(const node::mytopic::ConstPtr &msg){
        std::string log;
        std::string func_name = __FUNCTION__;
        func_name = ", called function: " + func_name;
        log = "Chatter get msg: " + msg->cmd.data + func_name;
        ROS_INFO("%s ",log.c_str());

        tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
                             ros::Time::now(),"base_link", "base_laser");



    }
    bool chat_service_cbk(node::myservice::Request& req,
                          node::myservice::Response& res){
        std::string log;
        std::string func_name = __FUNCTION__;
        func_name = ", called function: " + func_name;
        log = "Chatter service get msg: " + req.cmd.data+ func_name;
        ROS_INFO("%s ",log.c_str());
        res.success = true;
        return true;

    }

    void call_service(){
        // call service demo
        node::myservice srv;
        srv.request.cmd.data = "call service";
        ROS_INFO("call service");

        // call service
        //1)bare way  without nodehandle
        // ros::service::call("chat_service", srv)
        //2) with nodehandle
        if (chat_service__client_.exists()){
            ROS_INFO("exists service");

            if (chat_service__client_.call(srv))
            {
                ROS_INFO(" call service ok!: %ld", (long int)srv.response.success);
            }
            else
            {
                ROS_ERROR("Failed to call service set_filter");
            }
        }

        //with persistent services, you can tell if the connection failed by testing the handle:
        if (persistent_client_)
        {

        }
    }
};


// class member function callback
class Reflection_Detector{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // method
    //filter laserscan with tf

    //lookup tf

    // transform laser frame to map frame

    // get prepare configuration board position file


    // nearest neighbor search, fake board rejection

    // compute relative pose change

    // update map to odom tf

    // forward map_odom tf from amcl

};

class Listener{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    vector<ros::Subscriber> subscribers_;
    map<string, const ros::CallbackQueue &> callbackqueue_;
    bool updated_;
    node::mytopic tmp_data;
public:
    Listener(ros::NodeHandle nh,ros::NodeHandle nh_private);
    ~Listener(){};

    template <class T>
    boost::shared_ptr<T> createSubcriber(string topic, unsigned int buffer_size,boost::shared_ptr<T> &data);
//
    template <class T>
    std::shared_ptr<T>  getChat(string topic);

    template <class T>
    void callback(const typename T::ConstPtr &msg);

    template <class T>
    void bindcallback(const typename T::ConstPtr &msg, boost::shared_ptr<T> &data);
//
//    void callback(const sensor_msgs::LaserScanConstPtr &msg);
//    void callback(const node::mytopicConstPtr &msg);


};

Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nh_private) {
    updated_ = false;

}

template <class T>
boost::shared_ptr<T> Listener::createSubcriber(string topic, unsigned int buffer_size, boost::shared_ptr<T> &data) {
    boost::shared_ptr<node::mytopic> data_ptr = boost::shared_ptr<node::mytopic>(new T());

    ros::Subscriber chat_func_sub = nh_.subscribe<T>(topic, buffer_size, boost::bind(&Listener::bindcallback<T>, this,  _1,data));


//    ros::Subscriber chat_func_sub = nh_.subscribe(topic, 2, &Listener::callback<T>, this);

    subscribers_.push_back(chat_func_sub);

    boost::shared_ptr<T> p(boost::make_shared<T>(tmp_data));
    return p;

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

    ROS_INFO("receive msg,bind %s",msg->cmd.data.c_str());

    updated_ = true;
}
//void Listener::callback(const sensor_msgs::LaserScanConstPtr &msg) {
////    data = *msg;
//
//    updated_ = true;
//
//}
//void Listener::callback(const node::mytopicConstPtr &msg) {
////    data = *msg;
//
//    updated_ = true;

//}

int main(int argc, char **argv) {
    std::cout << "Hello, World!" << std::endl;
    //init a node
    ros::init(argc, argv, "start_up");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //**** topic
    // publish on o topic
    ros::Publisher chat_pub = nh.advertise<node::mytopic>("chat",1);
#if 0

    // listen on a topic,
    // 1)register a function as callback
    ros::Subscriber chat_func_sub = nh.subscribe("chat",2,chat_func_cbk);
    // 2)register class member function
    Chatter chatter(nh,nh_private);
    ros::Subscriber chat_class_sub = nh.subscribe("chat",2,&Chatter::chat_func_cbk, &chatter);
    // 3)register call member function in constructor, see Chatter constructor
#endif
    Listener l(nh,nh_private);
    std::shared_ptr<node::mytopic> p1 = l.getChat<node::mytopic>("chat");
    if (p1)
        ROS_INFO("%s",p1.get()->cmd.data.c_str());
    node::mytopic msg;
    boost::shared_ptr<node::mytopic> data = boost::shared_ptr<node::mytopic>(new node::mytopic());
    data->cmd.data = "sd";
    ros::spinOnce();
    l.createSubcriber<node::mytopic>("chat",2,data);

    ros::Rate rate(10);
    int i=0;
    while (ros::ok()){
        i++;
        chat_pub.publish(msg);


        char tmp[200];
        sprintf(tmp,"start %d",i);
        msg.cmd.data =string(tmp) ;
        ros::spinOnce();

        if(data){
            ROS_INFO("local receive %s",data.get()->cmd.data.c_str());
        }


        rate.sleep();
        ros::spinOnce();

    }

    // test code



    return 0;
}