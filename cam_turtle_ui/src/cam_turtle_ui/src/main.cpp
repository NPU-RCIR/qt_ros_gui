#include <QtGui>
#include <QApplication>
#include "../include/cam_turtle_ui/main_window.hpp"

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    cam_turtle_ui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
