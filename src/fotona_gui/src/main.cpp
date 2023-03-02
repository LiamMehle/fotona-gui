#include "main_window.h"
#include <QApplication>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "fotona_gui");
    ros::NodeHandle n;
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}