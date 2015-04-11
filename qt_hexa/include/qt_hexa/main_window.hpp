/**
 * @file /include/qt_hexa/main_window.hpp
 *
 * @brief Qt based gui for qt_hexa.
 *
 * @date November 2010
 **/
#ifndef qt_hexa_MAIN_WINDOW_H
#define qt_hexa_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_hexa {

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

    //void ReadSettings(); // Load up qt program settings at startup
    //void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();
    void sendMsg();
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    //void on_actionAbout_triggered();
    //void on_button_connect_clicked(bool check );
    //void on_checkbox_use_environment_stateChanged(int state);
    void on_horizontalSlider_sliderMoved(int position);
    void on_horizontalSlider_2_sliderMoved(int position);
    void on_horizontalSlider_3_sliderMoved(int position);

    void on_horizontalSlider_4_sliderMoved(int position);
    void on_horizontalSlider_5_sliderMoved(int position);
    void on_horizontalSlider_6_sliderMoved(int position);

    void on_pushButton_clicked();
    void on_spinBox_valueChanged(int value);

    void on_verticalSlider_sliderMoved(int position);
    void on_checkBox_clicked(bool check);

    void on_radioButton_clicked();
    void on_radioButton_2_clicked();
    void on_radioButton_3_clicked();
    /******************************************
    ** Manual connections
    *******************************************/
    //void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    ros::Publisher chatter_publisher;
    int pos[8];
    bool whileOk=false;
};

}  // namespace qt_hexa

#endif // qt_hexa_MAIN_WINDOW_H
