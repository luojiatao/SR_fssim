/**
 * @file /include/fssim_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef fssim_gui_QNODE_HPP_
#define fssim_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "fssim_common/CarInfo.h"
#include "fssim_common/Cmd.h"
#include <map>
#include <qprogressbar.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace fssim_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void car_info_signal(fssim_common::CarInfo); //car_info消息
    void dc_delta_signal(double, double); //dc和delta的量

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    /******************************************************************************************************************************************************
     * My Code Below ***************************************************************************************************************************************
     * ******************************************************************************************************************************************************/

private:
    //订阅车辆信息
    ros::Subscriber car_info_sub;

    //发布车辆控制信息
    ros::Publisher cmd_pub;

    //车辆控制信息
    double dc = 0;
    double delta = 0;

    //车辆控制限制
    const double dc_lim = 6;
    const double delta_lim = 0.5236;

    //function
    void car_info_cb(const fssim_common::CarInfo &msg);

public:
    //方向键状态
    std::map<char, bool> keyboard_press
    {
        {'W', false},
        {'S', false},
        {'A', false},
        {'D', false}
    };

    //车辆控制信号的最大值
    double dc_max = 1.0;
    double delta_max = 0.5236;
    //加速和转弯步长
    double dc_step = 1.0;
    double delta_step  = 0.05;
    //slider
    int slider_dc_step_pct = 0;
    int slider_delta_step_pct = 0;
    int slider_dc_max_pct = 0;
    int slider_delta_max_pct = 0;

    double get_dc_lim(){return this->dc_lim;}
    double get_delta_lim(){return this->delta_lim;}

    /******************************************************************************************************************************************************
     * My Code Above ---------------------------------------------------------------------------------------------------------------------------------------
     * ******************************************************************************************************************************************************/
};

}  // namespace fssim_gui

#endif /* fssim_gui_QNODE_HPP_ */
