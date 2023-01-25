#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>
#include <math.h>
#include <unistd.h>

#define MAX_LINEAR_VELOCITY 0.22l
#define MAX_ANGULAR_V 2.84


struct 
{
    double line_pos_x[2] = {0, 1};
    double line_pos_y[2] = {0, 0};
    double line_th[2] = {0, 0};

    double triangle_pos_x[4] = {0.577, 0.25, 0.41, 0.577};
    double triangle_pos_y[4] = {-0.557, -0.557, -0.27,  -0.577};
    double triangle_th[4] = { 3 * M_PI / 4,  M_PI / 2, 0,   3 * M_PI / 4};

    double square_pos_x[5] = {0.2, -0.11, -0.11, 0.2, 0.2};
    double square_pos_y[5] = {-0.26, -0.26, -0.575, -0.575, -0.26};
    double square_th[5] = {M_PI, M_PI, 0, M_PI};

    double circle_pos_x[9] = {-0.35, -0.225, -0.2, -0.225, -0.35, -0.43, -0.49, -0.43, -0.35};
    double circle_pos_y[9] = {-0.25, -0.325, -0.4, -0.5, -0.55, -0.5, -0.4, -0.325, -0.25};
    double circle_th[9] = {0, -M_PI/4, -M_PI/2, -3*M_PI/4, M_PI, 3*M_PI/4, M_PI/2, M_PI/4, 0};

} pose;

// void chatterCallback(const nav_msgs::Odometry::ConstPtr); // nur für ROS

class pid_controler
{
  public:
    pid_controler(){};
    ~pid_controler(){};

    double error(double current_x, double current_y, double current_th, double pos_x, double pos_y, double pos_th);

    double get_linear_velocity();

    double get_angular_velocity();

  private:
    double i_term;
    double d_term;
    double i_term_ang;
    double d_term_ang;
    double Ts = 0.1; // sample time
    double kp_lin = 0.3;
    double ki_lin = 0.1;
    double kd_lin = 0.1;
    double kp_alpha = 1.1;
    double kp_beta = -0.1;
    double ki_alpha = 0.1;
    double ki_beta = 0.1;
    double kd_alpha = 0.8;
    double kd_beta = 0.05;
    double delta_x;
    double delta_y;
    double rho;
    double old_rho;
    double alpha;
    double old_alpha;
    double beta;
    double old_beta;
    double velocity;
    double omega;
  };



  #endif