1.tf
1.1tf:static_transform_publisher

    static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
Publish a static coordinate transform to tf using an x/y/z offset in meters and yaw/pitch/roll in radians. (yaw is rotation about Z, pitch is rotation about Y, and roll is rotation about X). The period, in milliseconds, specifies how often to send a transform. 100ms (10hz) is a good value. 

    static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
Publish a static coordinate transform to tf using an x/y/z offset in meters and quaternion. The period, in milliseconds, specifies how often to send a transform. 100ms (10hz) is a good value. 


<launch>
<node pkg="tf" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1 100" />
</launch>


1.2.view_frames:

view_frames is a graphical debugging tool that creates a PDF graph of your current transform tree. 

    alias tf='cd /var/tmp && rosrun tf view_frames && evince frames.pdf &'
    
1.3.rqt-tf-tree
sudo apt-get install ros-kinetic-rqt-tf-tree

2.output any msg type
geometry_msgs/Point p;
ROS_INFO_STREAM(p)

3.create CallbackQueue
    
    ros::NodeHandle n;
    boost::shared_ptr<ros::CallbackQueue> q(boost::make_shared<ros::CallbackQueue>());
    n.setCallbackQueue(q.get());