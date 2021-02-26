#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/cam_turtle_ui/main_window.hpp"
#include <QDebug>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cam_turtle_ui {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this);
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::updateLogcamera(const QImage& img_)
{
//  std::cout<<"displayMat!!!"<<std::endl;
  qimage = img_.copy();
  /*这里应该设置中间变量qimage来储存img_，但是.copy()函数一用，程序就crashed，
   * 不知道是不是qt版本问题,------破案了，最终发现是用ros驱动的摄像头开启程序就会
   * 闪崩，而用m100_new里的cam包的pub_mono_cam节点发布摄像头就可以
   */
  qimage_mutex_.lock();
  ui.label_camera->setPixmap(QPixmap::fromImage(qimage));
  ui.label_camera->resize(ui.label_camera->pixmap()->size());
  qimage_mutex_.unlock();
}

void MainWindow::pose_status1(const QString& pose_x, const QString& pose_y)
{
  ui.xlabel->setText("turtle's position:("+pose_x+","+pose_y+")");
  if(pose_x.toDouble() > 5.54445 && pose_y.toDouble()>5.54445)
    ui.ylabel->setText("I'm in Right Up");
  else if(pose_x.toDouble()>5.54445 && pose_y.toDouble()<5.54445)
    ui.ylabel->setText("I'm in Right Down");
  else if(pose_x.toDouble()<5.54445 && pose_y.toDouble()>5.54445)
    ui.ylabel->setText("I'm in Left Up");
  else
    ui.ylabel->setText("I'm in Left Down");
}

void cam_turtle_ui::MainWindow::on_pushButton_clicked()
{
    timerId = this->startTimer(1000);
}

void cam_turtle_ui::MainWindow::timerEvent(QTimerEvent *event)
{
  static int sec = 0;
  std::cout<<sec<<std::endl;
  switch (sec) {
  case 1:
    system("gnome-terminal --geometry 80x8+55+452 -x bash -c 'source /opt/ros/kinetic/setup.bash; rosrun turtlesim turtlesim_node'&");
    //gnome-terminal打开终端，--geometry 80x8+55+452终端位置大小（终端输入xwininfo，指针变十字，点击终端即可获得终端的位置大小）rosrun turtlesim turtlesim_node
    system("gnome-terminal --geometry 80x24+55--10 -x bash -c 'source /opt/ros/kinetic/setup.bash; rosrun turtlesim turtle_teleop_key'&");
    //rosrun turtlesim turtle_teleop_key
    //system("gnome-terminal --geometry 80x24+55+14 -x bash -c 'source /opt/ros/kinetic/setup.bash; roslaunch usb_cam usb_cam-test.launch'&");
    //roslaunch usb_cam usb_cam-test.launch 程序会闪崩
    system("gnome-terminal --geometry 80x24+55+14 -x bash -c 'source ~/m100_new/devel/setup.bash; roslaunch cam pub_mono_cam.launch'&");
    //改用cam包的pub_mono_cam节点发布摄像头message
    break;
  case 3:
 /*这部分本来是在MainWindow的构造函数中，原本一点击start按钮，打开终端（注意，此时ros::subscriber已经开始订阅
 * 话题了（具体在init()函数中），但turtle终端都还没打开发布话题，导致发生错误），因此设置了延时控件，使得打开turtle终端发布话
 * 题后再运行init()函数开始订阅交互
 */
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
      QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

      /*********************
      ** Auto Start
      **********************/
      if ( ui.checkbox_remember_settings->isChecked() ) {
          on_button_connect_clicked(true);
      }
        if(!qnode.init()){
                showNoMasterMessage();
            }
      connect(&qnode,SIGNAL(loggingCamera(QImage)),this,
                       SLOT(updateLogcamera(const QImage)));

      connect(&qnode,SIGNAL(pose_status(QString, QString)),this,
              SLOT(pose_status1(const QString,const QString)));
      /*
       * 在qnode.cpp里有个类Qnode::qnode,且Qnode构造函数来订阅话题，因此当执行者qnode发射
       * 出pose_status(QString, QString)信号时（订阅到话题），响应者this(MainWindow)运
       * 行槽函数pose_status1(const QString,const QString)将传过来的参数赋值给Qlabel
      */
    break;
  case 7:
    this->killTimer(this->timerId);
    break;
  default:
    break;
  }
    sec++;
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "cam_turtle_ui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "cam_turtle_ui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace cam_turtle_ui

