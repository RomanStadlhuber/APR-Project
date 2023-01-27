#include <controller.hpp>
#include <iostream>
#include <math.h>
#include <unistd.h>


double pid_controler::error(double current_x, double current_y, double current_th, double pos_x, double pos_y, double pos_th) //error is stored in the class members
{
    delta_x = pos_x - current_x;
    delta_y = pos_y - current_y;

    old_rho = rho;
    rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    old_alpha = alpha;
    alpha = atan2(delta_y, delta_x) - current_th;
    if (alpha >= M_PI) alpha = -2*M_PI + alpha;
    if (alpha <= -M_PI) alpha = 2*M_PI + alpha;

    old_beta = beta;
    beta = - current_th - alpha - pos_th;
    if (beta >= M_PI) beta = -2*M_PI + beta;
    if (beta <= -M_PI) beta = -2*M_PI + beta;

    return rho; //returns the distanze to goal, to check if a goal is reached
}

double pid_controler::get_linear_velocity()
{
    double p_term = rho * kp_lin;
    i_term = i_term + ki_lin * rho * Ts;
    if (i_term > 5)
    i_term = 5;
    d_term = kd_lin * (rho - old_rho) / Ts;

    velocity = p_term; 

    if (velocity > MAX_LINEAR_VELOCITY)
    velocity = MAX_LINEAR_VELOCITY;

    return velocity;
}

double pid_controler::get_angular_velocity()
{
    double p_term_ang = kp_alpha * alpha + kp_beta * beta;
    i_term_ang = i_term_ang + ki_alpha * alpha * Ts + ki_beta * beta * Ts;
    if (i_term_ang > 5)
    i_term = 5;    
    d_term_ang = (kd_alpha * (alpha - old_alpha)) / Ts + (kd_beta * (beta - old_beta)) / Ts;


    omega = p_term_ang + d_term_ang;
    if (omega > MAX_ANGULAR_V)
    omega = MAX_ANGULAR_V;
    if (omega < -MAX_ANGULAR_V)
    omega = -MAX_ANGULAR_V;

    return omega;
}