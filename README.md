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

8.nodelet

9.plugins.



- used github code

- yaml
https://github.com/jimmiebergmann/mini-yaml.git
- kdtree
https://github.com/gishi523/kd-tree
- GRANSAC
https://github.com/drsrinathsridhar/GRANSAC.git
- Line fitting
http://durant35.github.io/2017/07/21/Algorithms_LeastSquaresLineFitting/
- optim
