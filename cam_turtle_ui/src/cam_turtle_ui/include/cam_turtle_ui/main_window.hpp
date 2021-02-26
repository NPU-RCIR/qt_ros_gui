/**
 * @file /include/cam_turtle_ui/main_window.hpp
 *
 * @brief Qt based gui for cam_turtle_ui.
 *
 * @date November 2010
 **/
#ifndef cam_turtle_ui_MAIN_WINDOW_H
#define cam_turtle_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QImage>
#include <QMutex>
#include <QLabel>
#include <QTime>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace cam_turtle_ui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
    void updateLogcamera(const QImage& img_);
    void pose_status1(const QString& pose_x, const QString& pose_y);
    void on_pushButton_clicked();

protected:
  void timerEvent(QTimerEvent *event);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QImage qimage;
    mutable QMutex qimage_mutex_;
    int timerId;
};

}  // namespace cam_turtle_ui

#endif // cam_turtle_ui_MAIN_WINDOW_H
