#include "main_window.h"
#include <QApplication>
#include <cstdio>
#include <exception>

int main(int argc, char *argv[]) {
	try {
		ros::init(argc, argv, "fotona_gui"); // small overhead
		ros::NodeHandle n;                   // likely best done on the main thread
		QApplication a(argc, argv);          // Qt stuff, must be on the main thread
		MainWindow w;
		w.show();
		return a.exec();
	} catch (ros::InvalidNodeNameException) {
		std::printf("[err]: Please reinstall fotona_gui, it is likely broken beyond repair.");
	} catch (std::bad_alloc) {
		std::printf("[err]: Failed to allocate enough memory.");
	} catch (std::runtime_error e) {
		std::printf("[err]: %s", e.what());
	}
}