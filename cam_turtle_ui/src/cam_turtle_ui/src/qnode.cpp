/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cam_turtle_ui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cam_turtle_ui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"cam_turtle_ui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    image_transport::ImageTransport it(n);
    image_sub = it.subscribe("usb_cam/image_raw",1000,&QNode::myCallback_img,this);//相机尝试
//     image_sub = it.subscribe("camera_BGR",1000,&QNode::myCallback_img,this);
    chatter_sub = n.subscribe<turtlesim::Pose>("turtle1/pose",1000,&QNode::chatterCallback,this); //新加-订阅的话题以及回调函数
    std::cout<<"init()"<<std::endl;
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"cam_turtle_ui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    image_transport::ImageTransport it(n);
	// Add your ros communications here.
   image_sub = it.subscribe("usb_cam/image_raw",1000,&QNode::myCallback_img,this);//相机尝试
//     image_sub = it.subscribe("camera_BGR",1000,&QNode::myCallback_img,this);
    chatter_sub = n.subscribe<turtlesim::Pose>("turtle1/pose",1000,&QNode::chatterCallback,this);//新加-订阅的话题以及回调函数
    std::cout<<"init(string string)"<<std::endl;
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(30);
	while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::chatterCallback(const turtlesim::Pose::ConstPtr &pose){
    double x, y;
    x = pose->x;
    y = pose->y;
    Q_EMIT pose_status(QString::number(x,'g',6),QString::number(y,'g',6));
    //订阅到消息后回调，发出pose_status信号
}

void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg)
{
//  std::cout<<"mycallback_img!!!"<<std::endl;
  try
  {
//    cv::imshow("gui_subscriber", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    img = cv_ptr->image;
    //QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);
    //img = cv_bridge::toCvShare(msg, "bgr8")->image;
    //ROS_INFO("I'm setting picture in mul_t callback function!");
    Q_EMIT loggingCamera(image);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

}  // namespace cam_turtle_ui
