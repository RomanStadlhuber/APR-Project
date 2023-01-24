#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <math.h>
#include <unistd.h>

#define MAX_LINEAR_VELOCITY 0.22
#define MAX_ANGULAR_V 2.84

double Xbot;
double Ybot;
double THbot;

void chatterCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  Xbot = msg->pose.pose.position.x;
  Ybot = msg->pose.pose.position.y;
  THbot = tf::getYaw(msg->pose.pose.orientation);
}

struct
{
  double start_pos_x = 0;
  double start_pos_y = 0;
  double start_th = 0;

  double line_pos_x[2] = {-0.499, 0.5};
  double line_pos_y[2] = {-0.241, -0.241};
  double line_th[2] = {0, 0};

  double triangle_pos_x[4] = {0.577, 0.25, 0.41, 0.577};
  double triangle_pos_y[4] = {-0.557, -0.557, -0.27,  -0.577};
  double triangle_th[4] = { 3 * M_PI / 4,  M_PI / 2, 0,   3 * M_PI / 4};

  double square_pos_x[5] = {0.2, -0.11, -0.11, 0.2, 0.2};
  double square_pos_y[5] = {-0.26, -0.26, -0.575, -0.575, -0.26};
  double square_th[5] = {M_PI, M_PI, 0, M_PI};

} pose;

class pid_controler
{
  public:
    pid_controler(){};
    ~pid_controler(){};

    double error(double current_x, double current_y, double current_th, double pos_x, double pos_y, double pos_th)
    {
      delta_x = pos_x - current_x;
      delta_y = pos_y - current_y;

      old_rho = rho;
      rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
      old_alpha = alpha;
      alpha = atan2(delta_y, delta_x) - current_th;
      if (alpha > M_PI) alpha = -M_PI;
      if (alpha < -M_PI) alpha = M_PI;
          
      old_beta = beta;
      beta = - current_th - alpha - pos_th;
      if (beta > M_PI) beta = M_PI;
      if (beta < -M_PI) beta = -M_PI;

      return rho;
    }

    double get_linear_velocity()
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

    double get_angular_velocity()
    {
      double p_term_ang = kp_alpha * alpha + kp_beta * beta;
      i_term_ang = i_term_ang + ki_alpha * alpha * Ts + ki_beta * beta * Ts;
      if (i_term_ang > 5)
        i_term = 5;    
      d_term_ang = (kd_alpha * (alpha - old_alpha)) / Ts + (kd_beta * (beta - old_beta)) / Ts;


      omega = p_term_ang + d_term_ang; // + d_term_ang;
      if (omega > MAX_ANGULAR_V)
        omega = MAX_ANGULAR_V;
      if (omega < -MAX_ANGULAR_V)
        omega = -MAX_ANGULAR_V;

      return omega;
    }

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

int main(int argc, char **argv)
{
  pid_controler PID;

  double current_pos_x = pose.start_pos_x;
  double current_pos_y = pose.start_pos_y;
  double current_th = pose.start_th;

  double v = 0;
  double omega = 0;

  ros::init(argc, argv, "odom_listener");
  ros::init(argc, argv, "talker");
  tf::TransformBroadcaster trans_boradcaster;
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);

  geometry_msgs::Twist velocity;

  double hz = 10;

  ros::Rate r(hz);

  int i = 0;

  // line

  while (i < 2)
  {

    current_pos_x = Xbot;
    current_pos_y = Ybot;
    current_th = THbot;


    v = PID.get_linear_velocity();
    omega = PID.get_angular_velocity();

    if (PID.error(current_pos_x, current_pos_y, current_th, pose.line_pos_x[i], pose.line_pos_y[i], pose.line_th[i]) < 0.08)
    {
      std::cout << "reached goal: " << i << "(" << pose.line_pos_x[i] << pose.line_pos_y[i] << ")" << std::endl;
      i++;
    }

    velocity.linear.x = v;
    velocity.angular.z = omega;

    pub.publish(velocity);

    ros::spinOnce();
    r.sleep();
  }

  std::cout << "line done" << std::endl;


  while (n.ok())
  {
    velocity.linear.x = 0;
    velocity.angular.z = 0;

    pub.publish(velocity);

    ros::spinOnce();
    r.sleep();
  }
  

  //---START---{"linear": 0.1, "angular": 0.10}___END___
  //---START---{"linear": 0.0, "angular": 0.00}___END___

  return 0;
}
