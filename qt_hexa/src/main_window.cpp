/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <ros/ros.h>
#include "../include/qt_hexa/main_window.hpp"
#include <std_msgs/Int64MultiArray.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_hexa {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
    , qnode(argc,argv)
{
    qnode.init();
    ui.setupUi(this);
    ros::NodeHandle n;
    chatter_publisher =n.advertise<std_msgs::Int64MultiArray>("body_kin",1000);
    for(int i=0;i<6;i++)
        pos[i]=0;
    pos[6]=1;
    pos[7]=1;
    // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    /*QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
    /*ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    /*if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }*/
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

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    pos[6]=0;
    pos[0]=position;
    sendMsg();
}

void MainWindow::on_horizontalSlider_2_sliderMoved(int position)
{
    pos[6]=0;
    pos[1]=position;
    sendMsg();
}

void MainWindow::on_horizontalSlider_3_sliderMoved(int position)
{
    pos[6]=0;
    pos[2]=position;
    sendMsg();
}

void MainWindow::on_horizontalSlider_4_sliderMoved(int position)
{
    pos[6]=0;
    pos[3]=position;
    sendMsg();
}

void MainWindow::on_horizontalSlider_5_sliderMoved(int position)
{
    pos[6]=0;
    pos[4]=position;
    sendMsg();
}

void MainWindow::on_horizontalSlider_6_sliderMoved(int position)
{
    pos[6]=0;
    pos[5]=position;
    sendMsg();
}

void MainWindow::on_verticalSlider_sliderMoved(int position)
{
    qnode.changedMode(position);
}

void MainWindow::on_spinBox_valueChanged(int value)
{
    pos[6]=value;
    printf("%d\n",value);
    sendMsg();
}

void MainWindow::on_checkBox_clicked(bool check)
{
    qnode.isBodyChecked(check);
}

void MainWindow::on_radioButton_clicked()
{
    pos[7]=1;
     pos[6]=0;
    sendMsg();
}

void MainWindow::on_radioButton_2_clicked()
{
    pos[7]=2;
     pos[6]=0;
    sendMsg();
}

void MainWindow::on_radioButton_3_clicked()
{
    pos[7]=3;
     pos[6]=0;
    sendMsg();
}

void MainWindow::on_pushButton_clicked()
{
    pos[6]=20001;
    if(ui.pushButton->text()=="Start")
        ui.pushButton->setText("Stop");
    else
        ui.pushButton->setText("Start");
    sendMsg();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */
void MainWindow::sendMsg()
{
    qnode.run(pos);
}

/*void MainWindow::on_button_connect_clicked(bool check ) {
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
}*/


/*void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}*/

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
/*void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}*/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

/*void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}*/

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

/*void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qt_hexa");
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
    QSettings settings("Qt-Ros Package", "qt_hexa");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}
*/
void MainWindow::closeEvent(QCloseEvent *event)
{
//	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace qt_hexa

