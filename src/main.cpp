//
// Created by waxz on 18-7-4.
//

#include <locate_reflection/boardFinder.h>
#include <locate_reflection/patternMatcher.h>
#include <cpp_utils/threading.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cpp_utils/svdlinefitting.h>
// for debug

#include <std_srvs/Empty.h>
#include "amcl/amcl_particles.h"

int main(int argc, char **argv) {


    std::cout << "Hello, World!" << std::endl;
    //init a node
    ros::init(argc, argv, "start_up");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    double rate;
    nh_private.param("rate", rate, 5.0);


    ros::Rate(1).sleep();


    ros::Rate r(rate);

    // construct a board Finder
    BoardFinder finder(nh, nh_private);
#if 0

    vector<Position> p1, p2;
    p1.push_back(Position(1,2,3.3,1,1));
#if 1
    p1.push_back(Position(-1,2,3.3,1,1));
    p1.push_back(Position(1.5,1.5,3.3,1,1));
    p1.push_back(Position(3,1,3.3,1,1));
#endif



    p2.push_back(Position(-3.1, 1.9,3.3,1,1));
    p2.push_back(Position( -1.1, 1.9,3.3,1,1));
    p2.push_back(Position( 0.9, 1.9,3.3,1,1));
    p2.push_back(Position(1.4, 1.4,3.3,1,1));
    finder.kdTreeMatch(p1,p2);

//    ros::Rate(1).sleep();
#endif
#if 1
    while (ros::ok()) {
        // detect board; ok
//        finder.detectBoard();
        // read xml board:ok
//        finder.getBoardPosition();
        // find location
        finder.findLocation();
        // kdtree match

        r.sleep();
    }


#endif

#if 0
    rosnode::Listener l(nh, nh_private);
    string service = "/amcl/set_particles";
    l.createServiceClient<amcl::amcl_particles>(service);
    amcl::amcl_particles srv;
    l.callService(service, srv);
    if (srv.response.success)
        cout<<"111";
    else
        cout<<"2222";
    cout<<srv.response.success<<endl;

    return 0;
#endif


#if 0
    // create a threading
        tf::TransformBroadcaster *tfb = new tf::TransformBroadcaster();

        threading_util::ThreadClass threadClass;

    threading_util::Threading t;
    ros::Duration transform_tolerance;
    transform_tolerance.fromSec(0.1);

    tf::Transform transform;
    ros::Time tn = ros::Time::now();
    ros::Time transform_expiration = (tn +
                                      transform_tolerance);

//            ROS_INFO("update odom by threads");
    tf::StampedTransform transformstamped(transform,
                                          transform_expiration,
                                          "map", "odom1");
    std::shared_ptr<tf::StampedTransform> data = std::make_shared<tf::StampedTransform>(transformstamped);
    data.get()->setIdentity();

//    threadClass.setTarget(data, tfb);

    threading_util::Func_tfb ff;
    ff.set(tfb);
    std::shared_ptr<threading_util::Func_tfb> fff = std::make_shared<threading_util::Func_tfb>(ff);



    threadClass.setTarget(ff, data);

    // test matcher
//    ros::Rate r(10);

    int i=1;
    geometry_msgs::Pose p;
    p.position.x = i;
    p.orientation.w =1;
    while (ros::ok() && i < 32) {
        if (i>5)
            threadClass.start();


        i++;
        p.position.x ++;
        tf::StampedTransform stampedTransform;
        stampedTransform.stamp_ = ros::Time::now();
        tf::poseMsgToTF(p,stampedTransform);
        std::swap(*data,stampedTransform );
        // ok
//        finder.detectBoard();
        r.sleep();
        cout<<"ddddd"<<endl;

    }


#endif

    // test matcher
#if 0
    PatternMatcher pm;

    vector<Position> p1, p2;
    p1.push_back(Position(-1,2,3.3));
    p1.push_back(Position(1,2,3.3));
    p1.push_back(Position(1.5,1.5,3.3));

    p2.push_back(Position(-3.1, 1.9,3.3));
    p2.push_back(Position( -1.1, 1.9,3.3));
    p2.push_back(Position( 0.9, 1.9,3.3));
    p2.push_back(Position(1.4, 1.4,3.3));

    pm.match(p1,p2);
#endif








}