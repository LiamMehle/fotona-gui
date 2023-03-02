#include "main_window.h"
#include <QApplication>
#include <cstdio>

int main(int argc, char *argv[]) {
	try {
		ros::init(argc, argv, "fotona_gui");
		ros::NodeHandle n;
		QApplication a(argc, argv);
		MainWindow w;
		w.show();
		return a.exec();
	} catch (ros::InvalidNodeNameException) {
		std::printf("Please reinstall fotona_gui, it is likely broken beyond repair.");
	} catch (std::bad_alloc) {
		std::printf("Failed to allocate enough memory.");
	}
}