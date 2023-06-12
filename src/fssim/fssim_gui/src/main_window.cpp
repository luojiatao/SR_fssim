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
#include "../include/fssim_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace fssim_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    /******************************************************************************************************************************************************
     * My Code Below ***************************************************************************************************************************************
     * ******************************************************************************************************************************************************/

    int init_dc_max_pct = 50;
    int init_delta_max_pct = 100;
    int init_dc_step_pct = 3;
    int init_delta_step_pct = 6;

    ui.horizontalSlider_dc_max->setValue(init_dc_max_pct);
    ui.horizontalSlider_delta_max->setValue(init_delta_max_pct);
    ui.horizontalSlider_dc_step->setValue(init_dc_step_pct);
    ui.horizontalSlider_delta_step->setValue(init_delta_step_pct);
    this->qnode.slider_dc_step_pct = init_dc_step_pct;
    this->qnode.slider_delta_step_pct = init_delta_step_pct;
    this->qnode.slider_dc_max_pct = init_dc_max_pct;
    this->qnode.slider_delta_max_pct = init_delta_max_pct;

    double dc_max = init_dc_max_pct / 100.0 * this->qnode.get_dc_lim();
    ui.label_dc_max_val->setText(QString::number(dc_max));
    this->qnode.dc_max = dc_max;

    double delta_max = init_delta_max_pct / 100.0 * this->qnode.get_delta_lim();
    ui.label_delta_max_val->setText(QString::number(delta_max));
    this->qnode.delta_max = delta_max;

    double dc_step = init_dc_step_pct / 100.0 * this->qnode.dc_max;
    ui.label_dc_step_val->setText(QString::number(dc_step));
    this->qnode.dc_step = dc_step;

    double delta_step = init_delta_step_pct / 100.0 * this->qnode.delta_max;
    ui.label_delta_step_val->setText(QString::number(delta_step));
    this->qnode.delta_step = delta_step;

    //链接qnode发送出来的car_info信号
    connect(&qnode, SIGNAL(car_info_signal(fssim_common::CarInfo)), this, SLOT(slot_update_car_info(fssim_common::CarInfo)));
    //链接qnode发送出来的dc和delta信息
    connect(&qnode, SIGNAL(dc_delta_signal(double, double)), this, SLOT(slot_update_dc_delta(double, double)));

    /******************************************************************************************************************************************************
     * My Code Above ---------------------------------------------------------------------------------------------------------------------------------------
     * ******************************************************************************************************************************************************/
}

MainWindow::~MainWindow() {}



/******************************************************************************************************************************************************
 * My Code Above ---------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************************************************************************************************************/

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
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
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "fssim_gui");
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
    QSettings settings("Qt-Ros Package", "fssim_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace fssim_gui


/******************************************************************************************************************************************************
 * My Code Below ***************************************************************************************************************************************
 * ******************************************************************************************************************************************************/

void fssim_gui::MainWindow::slot_update_car_info(fssim_common::CarInfo car_info){
    ui.label_dc_val->setNum(car_info.dc);
    ui.label_delta_val->setNum(car_info.delta);
    ui.label_front_left_steering_angle_val->setNum(car_info.front_left_steering_angle);
    ui.label_front_right_steering_angle_val->setNum(car_info.front_right_steering_angle);
    ui.label_delta_measured_val->setNum(car_info.delta_measured);
    ui.label_vx_val->setNum(car_info.vx);
    ui.label_vy_val->setNum(car_info.vy);
    ui.label_r_val->setNum(car_info.r);
    ui.label_alpha_f_val->setNum(car_info.alpha_f);
    ui.label_alpha_f_l_val->setNum(car_info.alpha_f_l);
    ui.label_alpha_f_r_val->setNum(car_info.alpha_f_r);
    ui.label_alpha_r_l_val->setNum(car_info.alpha_r_l);
    ui.label_alpha_r_val->setNum(car_info.alpha_r);
    ui.label_alpha_r_r_val->setNum(car_info.alpha_r_r);
    ui.label_Fy_f_val->setNum(car_info.Fy_f);
    ui.label_Fy_f_l_val->setNum(car_info.Fy_f_l);
    ui.label_Fy_f_r_val->setNum(car_info.Fy_f_r);
    ui.label_Fy_r_val->setNum(car_info.Fy_r);
    ui.label_Fy_r_l_val->setNum(car_info.Fy_r_l);
    ui.label_Fy_r_r_val->setNum(car_info.Fy_r_r);
    if(car_info.torque_ok){
        ui.checkbox_torque_ok->setCheckState(Qt::CheckState::Checked);
    }else{
        ui.checkbox_torque_ok->setCheckState(Qt::CheckState::Unchecked);
    }
}

void fssim_gui::MainWindow::on_horizontalSlider_dc_step_valueChanged(int value)
{
    this->qnode.slider_dc_step_pct = value;
    double dc_step = value / 100.0 * this->qnode.dc_max;
    ui.label_dc_step_val->setNum(dc_step);
    this->qnode.dc_step = dc_step;
}

void fssim_gui::MainWindow::on_horizontalSlider_delta_step_valueChanged(int value)
{
    this->qnode.slider_delta_step_pct = value;
    double delta_step = value / 100.0 * this->qnode.delta_max;
    ui.label_delta_step_val->setNum(delta_step);
    this->qnode.delta_step = delta_step;
}

void fssim_gui::MainWindow::on_horizontalSlider_dc_max_valueChanged(int value)
{
    this->qnode.slider_dc_max_pct = value;
    double dc_max = value / 100.0 * this->qnode.get_dc_lim();
    ui.label_dc_max_val->setNum(dc_max);
    this->qnode.dc_max = dc_max;

    double dc_step = this->qnode.slider_dc_step_pct / 100.0 * this->qnode.dc_max;
    ui.label_dc_step_val->setNum(dc_step);
    this->qnode.dc_step = dc_step;
}

void fssim_gui::MainWindow::on_horizontalSlider_delta_max_valueChanged(int value)
{
    this->qnode.slider_delta_max_pct = value;
    double delta_max = value / 100.0 * this->qnode.get_delta_lim();
    ui.label_delta_max_val->setNum(delta_max);
    this->qnode.delta_max = delta_max;

    double delta_step = this->qnode.slider_delta_step_pct / 100.0 * this->qnode.delta_max;
    ui.label_delta_step_val->setNum(delta_step);
    this->qnode.delta_step = delta_step;
}

void fssim_gui::MainWindow::slot_update_dc_delta(double dc, double delta){
    ui.label_cmd_dc_val->setNum(dc);
    ui.label_cmd_delta_val->setNum(delta);

    ui.progressBar_cmd_dc->setValue(dc / this->qnode.dc_max * 50 + 50);
    ui.progressBar_cmd_delta->setValue(-delta / this->qnode.delta_max * 50 + 50);

}

//键盘按下事件
void fssim_gui::MainWindow::keyPressEvent(QKeyEvent * event)
{
    switch (event->key())
    {
        case Qt::Key_W:
            if(!event->isAutoRepeat()){
                this->qnode.keyboard_press['W']=true;
            }
            break;
        case Qt::Key_S:
            if(!event->isAutoRepeat()){
                this->qnode.keyboard_press['S']=true;
            }
            break;
        case Qt::Key_A:
            if(!event->isAutoRepeat()){
                this->qnode.keyboard_press['A']=true;
            }
            break;
        case Qt::Key_D:
            if(!event->isAutoRepeat()){
                this->qnode.keyboard_press['D']=true;
            }
            break;
    }
}

//键盘松开事件
void fssim_gui::MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    switch (event->key())
    {
        case Qt::Key_W:
            if(!event->isAutoRepeat()){
                this->qnode.keyboard_press['W']=false;
            }
        break;
        case Qt::Key_S:
            if(!event->isAutoRepeat()){
                this->qnode.keyboard_press['S']=false;
            }
        break;
        case Qt::Key_A:
            if(!event->isAutoRepeat()){
                this->qnode.keyboard_press['A']=false;
            }
        break;
        case Qt::Key_D:
            if(!event->isAutoRepeat()){
                this->qnode.keyboard_press['D']=false;
            }
        break;
    }
}
