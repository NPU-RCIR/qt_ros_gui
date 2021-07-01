# qt_ros_gui
this is a demo about qt_ros_gui, it shows image topic(sensor_msgs::Image type) and other topic types in a QROSApp UI

Env:Ubuntu 16.04
Qt5.9.1 creator with qtcreator-plugin-ros
ros(kinetic)

Step 1.
* mkdir -p ~/catkin_ws/src
* cd ~/catkin_ws/src/
* git clone https://github.com/NPU-RCIR/qt_ros_gui.git
* cd ..
* catkin_make

 if error occured like :"Parse error at "BOOST_JOIN""
 * Solution: https://blog.csdn.net/u011573853/article/details/103071178

Step 2.
run the executable file in $workspace/devel/lib/$pkg_name
* double click or ./$executable_file

Step 3.result
![20210226215301](https://user-images.githubusercontent.com/66043885/109308545-20b65f00-787d-11eb-96c6-b75b48ada358.png)
