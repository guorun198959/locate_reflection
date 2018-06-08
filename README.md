reflection_localization


develop as a template ros cmake startup project

1.introduce dependency in CMakeList.txt and package.xml
sensor_msgs geometry_msgs nav_msgs tf roscpp
2.generate custom msg and srv in CMakeList.txt

3.define topic and service callback in node.cpp, call service

4.use message filter, define callback

5.tf listener lookup , update and  transformation, develop a easy use tf transform lib

6.message callback queue

7.mutil threading
