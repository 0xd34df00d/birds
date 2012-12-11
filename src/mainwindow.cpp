#include "mainwindow.h"
#include <QTimer>
#include "glwidget.h"

MainWindow::MainWindow ()
: QMainWindow ()
{
	W_ = new GLWidget (this);
	setCentralWidget (W_);
	QTimer::singleShot (200, this, SLOT (doStuff ()));
}

void MainWindow::doStuff ()
{
	W_->build (QString::fromUtf8 ("data/п_1.txt"));
}
