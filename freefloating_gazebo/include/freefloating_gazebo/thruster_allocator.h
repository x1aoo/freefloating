#ifndef THRUSTER_ALLOCATOR_H
#define THRUSTER_ALLOCATOR_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <freefloating_gazebo/hydro_model_parser.h>
#include <visp/vpFeaturePoint.h>
// #include <vector>

std::vector<double> plot_cost_x, plot_cost_y, plot_cost_z, plot_cost_roll, plot_cost_pitch, plot_cost_yaw, plot_cost_sum, plot_cost_roll_square, plot_cost_rate;
std::vector<double> plot_time;

namespace ffg
{

class ThrusterAllocator
{
public:
  // parse raw param to get thruster max force and map
  ThrusterAllocator(ros::NodeHandle &nh);



  std::vector<std::string> initControl(ros::NodeHandle &nh, double map_threshold = 1e-2);//to push the controlled axes to the nh and map_threshold is useless
  bool has_thrusters() const {return names.size();}

  //transfor the array to the formolized message
  geometry_msgs::Wrench maxWrench() const//where we use the maxWrench function or?
  {
    geometry_msgs::Wrench wrench;
    wrench.force.x = max_wrench[0];
    wrench.force.y = max_wrench[1];
    wrench.force.z = max_wrench[2];
    wrench.torque.x = max_wrench[3];
    wrench.torque.y = max_wrench[4];
    wrench.torque.z = max_wrench[5];
    return wrench;
  }


  void saturate(Eigen::VectorXd &_command) const;

  sensor_msgs::JointState wrench2Thrusters(const geometry_msgs::Wrench  & cmd, double angle1, double angle2, double fl, double fr, double f2, double f3, double f4, ros::NodeHandle &nh) const;
//  std::vector<sensor_msgs::JointState> wrench2Thrusters_iterative(const geometry_msgs::Wrench  & cmd, vpColVector state_pre, ros::NodeHandle &nh) const;
    sensor_msgs::JointState wrench2Thrusters_iterative(const geometry_msgs::Wrench  & cmd, vpColVector state_pre, ros::NodeHandle &nh) const;
    sensor_msgs::JointState wrench2Thrusters_3rd(const geometry_msgs::Wrench &cmd, vpColVector state_pre, vpColVector state_poi, vpColVector state_vel, ros::NodeHandle &nh) const;


    ffg::HydroLink base_link;
  std::vector<uint> steer_idx, fixed_idx;
  std::vector<std::string> names;//pase from passer
  std::vector<double> max_thrust, max_wrench, max_vel;

  Eigen::MatrixXd map;
  Eigen::MatrixXd inverse_map;





};

}



#endif // THRUSTER_ALLOCATOR_H
