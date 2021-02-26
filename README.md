# qt_ros_gui
this is a demo about qt_ros_gui, it shows camera messages and other common ros data in a QROSApp UI

Step 1.
（Make sure your Qt is installed with the “qtcreator-plugin-ros”）
* mkdir -p ~/catkin_ws/src
* cd ~/catkin_ws/src/
* catkin_create_qt_pkg cam_turtle_ui std_msgs rospy roscpp sensor_msgs cv_bridge image_transport

Step 2.
we need launch pub_mono_cam node in cam package(a sub package in m100_new), not the usb_cam package comes with ros

Step 3.result
![20210226215301](https://user-images.githubusercontent.com/66043885/109308545-20b65f00-787d-11eb-96c6-b75b48ada358.png)
