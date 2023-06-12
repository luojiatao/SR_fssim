/**
 * @file /include/fssim_gui/main_window.hpp
 *
 * @brief Qt based gui for fssim_gui.
 *
 * @date November 2010
 **/
#ifndef fssim_gui_MAIN_WINDOW_H
#define fssim_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "ros/ros.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace fssim_gui {

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

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void slot_update_car_info(fssim_common::CarInfo);

    void slot_update_dc_delta(double, double);

    void on_horizontalSlider_dc_step_valueChanged(int value);

    void on_horizontalSlider_delta_step_valueChanged(int value);

    void on_horizontalSlider_dc_max_valueChanged(int value);

    void on_horizontalSlider_delta_max_valueChanged(int value);

    /******************************************************************************************************************************************************
     * My Code Below ***************************************************************************************************************************************
     * ******************************************************************************************************************************************************/

protected:
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);

    /******************************************************************************************************************************************************
     * My Code Above ---------------------------------------------------------------------------------------------------------------------------------------
     * ******************************************************************************************************************************************************/


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace fssim_gui

#endif // fssim_gui_MAIN_WINDOW_H
