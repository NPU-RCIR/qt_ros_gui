/**
 * @file /include/cam_turtle_ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cam_turtle_ui_QNODE_HPP_
#define cam_turtle_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>
#include <sensor_msgs/image_encodings.h>
#include </opt/ros/kinetic/include/turtlesim/Pose.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cam_turtle_ui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
    void myCallback_img(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat img;
    QImage image;

Q_SIGNALS:
    void rosShutdown();
    void loggingCamera(QImage);
    void pose_status(QString, QString);
private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Subscriber chatter_subscriber;
    void chatterCallback(const turtlesim::Pose::ConstPtr &pose);
    QStringListModel logging_model;
    ros::Subscriber chatter_sub;
    image_transport::Subscriber image_sub;
};

}  // namespace cam_turtle_ui

#endif /* cam_turtle_ui_QNODE_HPP_ */
