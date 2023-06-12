/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/fssim_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace fssim_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"fssim_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    /******************************************************************************************************************************************************
     * My Code Below ***************************************************************************************************************************************
     * ******************************************************************************************************************************************************/

    //解决报错
    qRegisterMetaType<QVector<int>>("QVector<int>");
    //注册自定义的msg类型
    qRegisterMetaType<fssim_common::CarInfo>("fssim_common::CarInfo");
    qRegisterMetaType<fssim_common::Cmd>("fssim_common::Cmd");

    //订阅车辆信息
    this->car_info_sub = n.subscribe("/fssim/car_info", 1000, &QNode::car_info_cb, this);
    //发布车辆控制信息
    this->cmd_pub = n.advertise<fssim_common::Cmd>("/fssim/cmd", 1000);

    /******************************************************************************************************************************************************
     * My Code Above ---------------------------------------------------------------------------------------------------------------------------------------
     * ******************************************************************************************************************************************************/

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"fssim_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    /******************************************************************************************************************************************************
     * My Code Below ***************************************************************************************************************************************
     * ******************************************************************************************************************************************************/

    //解决报错
    qRegisterMetaType<QVector<int>>("QVector<int>");

    //订阅车辆信息
    this->car_info_sub = n.subscribe("/fssim/car_info", 1000, &QNode::car_info_cb, this);

    /******************************************************************************************************************************************************
     * My Code Above ---------------------------------------------------------------------------------------------------------------------------------------
     * ******************************************************************************************************************************************************/

	start();
	return true;
}

/******************************************************************************************************************************************************
 * My Code Below ***************************************************************************************************************************************
 * ******************************************************************************************************************************************************/

void QNode::car_info_cb(const fssim_common::CarInfo &msg){
    emit car_info_signal(msg);
}

/******************************************************************************************************************************************************
 * My Code Above ---------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************************************************************************************************************/


void QNode::run() {
    ros::Rate loop_rate(100);
	int count = 0;
	while ( ros::ok() ) {

        /******************************************************************************************************************************************************
         * My Code Below ***************************************************************************************************************************************
         * ******************************************************************************************************************************************************/

        if(this->keyboard_press['W']){
            //如果前进
            if(this->dc < 0){
                //如果反向加速
                this->dc = this->dc + this->dc_step * 2;
            }else{
                //如果普通加速
                this->dc = this->dc + this->dc_step;
            }
            //限幅
            if(this->dc >= this->dc_max) this->dc = this->dc_max;
            if(this->dc <= -this->dc_max) this->dc = -this->dc_max;
            //if(this->dc <= 0) this->dc = 0;
        }else if(this->keyboard_press['S']){
            //如果后退
            if(this->dc > 0){
                //如果反向加速
                this->dc = this->dc - this->dc_step * 2;
            }else{
                //如果普通加速
                this->dc = this->dc - this->dc_step;
            }
            //限幅
            if(this->dc >= this->dc_max) this->dc = this->dc_max;
            if(this->dc <= -this->dc_max) this->dc = -this->dc_max;
            //if(this->dc <= 0) this->dc = 0;
        }else{
            //如果既不前进，也不后退
            if(this->dc > 0){
                this->dc = this->dc - this->dc_step;
            }else{
                this->dc = this->dc + this->dc_step;
            }
            if(fabs(this->dc) <= this->dc_step){
                this->dc = 0;
            }
        }

        if(this->keyboard_press['A']){
            if(this->delta < 0){
                //如果反向转向
                this->delta = this->delta + this->delta_step * 2;
            }else{
                //如果同向转向
                this->delta = this->delta + this->delta_step;
            }
            if(this->delta >= this->delta_max) this->delta = this->delta_max;
            if(this->delta <= -this->delta_max) this->delta = -this->delta_max;
        }else if(this->keyboard_press['D']){
            if(this->delta > 0){
                //如果反向转向
                this->delta = this->delta - this->delta_step * 2;
            }else{
                //如果正向转向
                this->delta = this->delta - this->delta_step;
            }
            if(this->delta >= this->delta_max) this->delta = this->delta_max;
            if(this->delta <= -this->delta_max) this->delta = -this->delta_max;
        }else{
            if(this->delta > 0){
                this->delta = this->delta - this->delta_step;
            }else{
                this->delta = this->delta + this->delta_step;
            }
            if(fabs(this->delta) <= this->delta_step){
                this->delta = 0;
            }
        }

        //将dc和delta发送到mainWindow显示
        emit dc_delta_signal(this->dc, this->delta);

        fssim_common::Cmd cmd;
        cmd.dc = this->dc;
        cmd.delta = this->delta;
        this->cmd_pub.publish(cmd);

        /*
        if(this->ptr_progressBar_dc != nullptr){
            this->ptr_progressBar_dc->setValue(this->dc / this->dc_max * 100);
        }
        if(this->ptr_progressBar_delta != nullptr){
            this->ptr_progressBar_delta->setValue(-this->delta / this->delta_max * 100 + 50);
        }
        */

        /******************************************************************************************************************************************************
         * My Code Above ---------------------------------------------------------------------------------------------------------------------------------------
         * ******************************************************************************************************************************************************/

        /*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
        */

		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace fssim_gui
