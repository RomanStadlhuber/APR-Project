#include <iostream>
#include <math.h>
#include <unistd.h>  


#define MAX_LINEAR_VELOCITY 0.22
#define MAX_ANGULAR_V 2.84




struct{
    double start_pos_x = 0;
    double start_pos_y = 0;
    double start_th = 0;

    double line_pos_x[2] = {5, 5};
    double line_pos_y[2] = {5, 10};
    double line_th[2] = {0, 0};

    double square_pos_x[4] = {1, 2, 3, 4};
    double square_pos_y[4] = {1, 2, 3, 4};
    double square_th[4] = {M_PI/4, -M_PI/4, -3*M_PI/4, -3*M_PI/4};

    double triangle_pos_x[3] = {1, 2, 3};
    double triangle_pos_y[3] = {1, 2, 3};
    double triangle_th[3] = {0, 0, 0};
} pose;


class pid_controler
{
    public:
        pid_controler(/* args */){};
        ~pid_controler(){};

        double error(double current_x, double current_y, double current_th, double pos_x, double pos_y, double pos_th)
        {
            delta_x = pos_x - current_x;
            delta_y = pos_y - current_y;

            old_rho = rho;
            rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
            old_alpha = alpha;
            alpha = atan2(delta_y, delta_x) - current_th;
            old_beta = beta;
            beta = -current_th - alpha - pos_th;

            std::cout << "rho = " << rho << std::endl;

            return rho;
        }
        

        double get_linear_velocity()
        {            
            double p_term = rho * kp_lin;
            i_term = i_term + ki_lin * rho;
            if(i_term < 100) i_term = 100;
            d_term = kd_lin * (rho - old_rho);

            velocity = p_term + d_term + i_term;
            
            if(velocity > MAX_LINEAR_VELOCITY) velocity = MAX_LINEAR_VELOCITY;
  
            return velocity;
        }

        double get_angular_velocity()
        {
            double p_term_ang = kp_alpha * alpha + kp_beta * beta;
            i_term_ang = i_term_ang + ki_alpha * alpha + ki_beta * beta;
            d_term_ang = kd_alpha * (alpha - old_alpha) + kd_beta * (beta - old_beta);

            omega = p_term_ang + i_term_ang + d_term_ang;
            if(omega > MAX_ANGULAR_V) omega = MAX_ANGULAR_V;
                
            return omega;
        }

    private:
        double i_term;
        double d_term;
        double i_term_ang;
        double d_term_ang;
        double Ts = 0.1; //sample time
        double kp_lin = 0.4;
        double ki_lin = 0.1;
        double kd_lin = 0.1;
        double kp_alpha = 0.8;
        double kp_beta = -0.1;
        double ki_alpha = 0.1;
        double ki_beta = 0.1;
        double kd_alpha = 0.1;
        double kd_beta = 0.1;
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




int main()
{
    pid_controler PID;

    double current_pos_x = pose.start_pos_x;
    double current_pos_y = pose.start_pos_y;
    double current_th = pose.start_th;

    int i = 0;

    double v = 0;
    double omega = 0;

    //line

    while (i < 2)
    {
        if(PID.error(current_pos_x, current_pos_y, current_th, pose.line_pos_x[i], pose.line_pos_y[i], pose.line_th[i]) < 0.5){
            i++;
            std::cout << "ifif" << std::endl;
        }
        v = PID.get_linear_velocity();
        omega = PID.get_angular_velocity();
        std::cout << "linear = " << v << " angular = " << omega << std::endl;
        //get updated pos
        sleep(1);
    }
    

    //---START---{"linear": 0.1, "angular": 0.10}___END___
    //---START---{"linear": 0.0, "angular": 0.00}___END___

    //square
    /*
    while (i < 4)
    {
        if(PID.error(current_pos_x, current_pos_y, current_th, pose.square_pos_x[i], pose.square_pos_y[i], pose.square_th[i]) < 0.5){
            i++;
        }
        v = PID.get_angular_velocity();
        omega = PID.get_linear_velocity();
        std::cout << "linear = " << v << "angular = " << omega << std::endl;
    }
    

    while (i < 3)
    {
        if(PID.error(current_pos_x, current_pos_y, current_th, pose.triangle_pos_x[i], pose.triangle_pos_y[i], pose.triangle_th[i]) < 0.5){
            i++;
        }
        v = PID.get_angular_velocity();
        omega = PID.get_linear_velocity();
        std::cout << "linear = " << v << "angular = " << omega << std::endl;
    }
    */




    return 0;
}