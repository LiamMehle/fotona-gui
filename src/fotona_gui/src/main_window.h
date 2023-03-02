#pragma once

#include <memory>

#include <QMainWindow>
#include <QPushButton>
#include <QGridLayout>

// #include <QColor>
// #include <QSlider>
// #include <QLabel>
// #include <QVBoxLayout>

// visualization_manager.h uses `register`
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include "rviz/visualization_manager.h"
#pragma GCC diagnostic pop
#include "rviz/render_panel.h"
#include "rviz/display.h"



// todo: remove
// QT_BEGIN_NAMESPACE
// namespace Ui { class MainWindow; }
// QT_END_NAMESPACE

class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();

private:
	QPushButton* clear_button;
	QPushButton* scan_button;
	QPushButton* start_button;
	QPushButton* stop_button;
	QGridLayout* main_layout;
	QWidget*     central_widget;

	std::unique_ptr<rviz::VisualizationManager> manager;
	rviz::RenderPanel*             render_panel;
	// `Display` in this context meaning something that is shown or element in the reneder pannel.
	rviz::Display* grid;
};