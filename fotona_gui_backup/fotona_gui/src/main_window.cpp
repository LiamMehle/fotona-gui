#include "main_window.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

	this->render_panel = std::unique_ptr<rviz::RenderPanel>(new rviz::RenderPanel(parent));

// set button text
	this->clear_button.setText("clear");
	this->scan_button.setText("scan");
	this->start_button.setText("start");
	this->stop_button.setText("stop it, get some help");

	this->connect(&this->clear_button, &QPushButton::clicked, this,
		[this](){ puts("button has been clicked!"); });

// layout setup
	this->main_layout.addWidget(&this->clear_button,       0, 0, 1, 1);
	this->main_layout.addWidget(&this->scan_button,        1, 0, 1, 1);
	this->main_layout.addWidget(&this->start_button,       2, 0, 1, 1);
	this->main_layout.addWidget(&this->stop_button,        3, 0, 1, 1);
	this->main_layout.addWidget( this->render_panel.get(), 0, 1, 4, 1);

	this->central_widget.setLayout(&main_layout);
	this->setCentralWidget(&central_widget);

// fix up default layout
#define FIX(WIDGET) this->WIDGET.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding)
	FIX(clear_button);
	FIX(scan_button);
	FIX(start_button);
	FIX(stop_button);
#undef FIX

#define FOR(VAR, RANGE) for(int VAR=0; VAR<RANGE; VAR++)
	FOR (i, this->main_layout.rowCount())
		this->main_layout.setRowStretch(i, 1);
	// FOR (j, this->main_layout.columnCount())
	// 	this->main_layout.setColumnStretch(j, 1);
#undef FOR
	this->main_layout.setColumnStretch(0, 1);
	this->main_layout.setColumnStretch(1, 4);

}

MainWindow::~MainWindow() {
	// delete this->ui;
}
