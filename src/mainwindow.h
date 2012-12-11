#pragma once

#include <QMainWindow>

class GLWidget;

class MainWindow : public QMainWindow
{
	Q_OBJECT

	GLWidget *W_;
public:
	MainWindow ();
private slots:
	void doStuff ();
};
