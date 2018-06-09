//
// Created by waxz on 18-6-9.
//
#include <ros/ros.h>

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


#if 0

// listen on a topic,
    // 1)register a function as callback
    ros::Subscriber chat_func_sub = nh.subscribe("chat",2,chat_func_cbk);
    // 2)register class member function
    Chatter chatter(nh,nh_private);
    ros::Subscriber chat_class_sub = nh.subscribe("chat",2,&Chatter::chat_func_cbk, &chatter);
    // 3)register call member function in constructor, see Chatter constructor
#endif