#ifndef CAR_INFO_MSG_HPP
#define CAR_INFO_MSG_HPP

struct CarInfoMsg{
    double delta;
    double dc;

    double front_left_steering_angle;
    double front_right_steering_angle;
    double delta_measured;

    double vx;
    double vy;
    double r;

    bool torque_ok;

    double alpha_f;
    double alpha_f_l;
    double alpha_f_r;
    double alpha_r_l;
    double alpha_r;
    double alpha_r_r;

    double Fy_f;
    double Fy_f_l;
    double Fy_f_r;
    double Fy_r;
    double Fy_r_l;
    double Fy_r_r;

    double Fx;
};

#endif //CAR_INFO_MSG_HPP