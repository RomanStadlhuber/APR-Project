#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>
#include <math.h>
#include <unistd.h>

#define MAX_LINEAR_VELOCITY 0.1
#define MAX_ANGULAR_V 2.84


struct 
{
    double line_pos_x[2] = {0, 1};
    double line_pos_y[2] = {0, 0};
    double line_th[2] = {0, 0};

    //                         {P1  P2        P3                P4      }
    double triangle_pos_x[4] = {0,  0.3,      0.22,             0       };
    double triangle_pos_y[4] = {0,  0,        0.26,             0       };
    double triangle_th[4]    = {0,  -0.424,   -0.424-(M_PI/2),  2.292   };

    //                         P1   P2     P3                P4        P5
    double square_pos_x[5]   = {0,  0.29,  0.29,             0,        0};
    double square_pos_y[5]   = {0,  0,     -0.30,            -0.30,    0};
    double square_th[5]      = {0,  0,     (M_PI/2),         -M_PI,     -(M_PI/2)};

    //
    double circle_pos_x[9]  = {0, 0.1025,    0.145,     0.1025,     0,      -0.1025,    -0.145,   -0.1025,   0};
    double circle_pos_y[9]  = {0, -0.0424,   -0.145,    -0.2475,    -0.29,  -0.2475,    -0.145,   -0.0424,   0};
    double circle_th[9]     = {0, -M_PI/4,   -M_PI/2,   -3*M_PI/4,  M_PI,   3*M_PI/4,   M_PI/2,   M_PI/4,    0};

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
    double kp_alpha = 1.4;
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