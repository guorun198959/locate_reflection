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
// for debug

template<class Vector3>
std::pair<Vector3, Vector3> best_line_from_points(const std::vector<Vector3> &c) {
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix<typename Vector3::Scalar, Eigen::Dynamic, Eigen::Dynamic> centers(num_atoms, 3);
    for (size_t i = 0; i < num_atoms; ++i) centers.row(i) = c[i];

    Vector3 origin = centers.colwise().mean();
    Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Vector3 axis = eig.eigenvectors().col(2).normalized();

    return std::make_pair(origin, axis);
}



int main(int argc, char **argv) {


    std::cout << "Hello, World!" << std::endl;
    //init a node
    ros::init(argc, argv, "start_up");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    tf::TransformBroadcaster *tfb = new tf::TransformBroadcaster();


    // construct a board Finder
    BoardFinder finder(nh, nh_private);

    ros::Rate r(10);

    threading_util::ThreadClass threadClass;


#if 1
    while (ros::ok()) {
        // detect ok;
//        finder.detectBoard();
        // find location
//        finder.findLocation();
        r.sleep();
    }


#endif
#if 0
    // create a threading
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
                                          "map", "odom");
    std::shared_ptr<tf::StampedTransform> data = std::make_shared<tf::StampedTransform>(transformstamped);
    data.get()->setIdentity();

//    threadClass.setTarget(data, tfb);

    threading_util::Func_tfb ff;
    ff.set(tfb);
    std::shared_ptr<threading_util::Func_tfb> fff = std::make_shared<threading_util::Func_tfb>(ff);


#if 0


#endif
    threadClass.setTarget(ff, data);

    // test matcher
//    ros::Rate r(10);

    int i=1;
    geometry_msgs::Pose p;
    p.position.x = i;
    p.orientation.w =1;
    while (ros::ok() && i < 32) {
        if (i>20)
            threadClass.start();


        i++;
        p.position.x ++;
        tf::poseMsgToTF(p,*data);
        // ok
//        finder.detectBoard();
        data.get()->stamp_ = ros::Time::now();
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