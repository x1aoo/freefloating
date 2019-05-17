#include <freefloating_gazebo/freefloating_pids_body.h>
#include </home/x1ao/master/master_thesis_auv/ros_auv/src/freefloating_gazebo/src/matplotlibcpp.h>

namespace plt = matplotlibcpp;

using std::cout;
using std::endl;
using std::string;

//subscribe and run the callback funcion, have the control axis and relative command and the realative controller
void FreeFloatingBodyPids::Init(ros::NodeHandle &nh, ros::Duration&_dt,
                                const std::vector<std::string>&_controlled_axes,
                                std::string default_mode)
{
  // init dt from rate
  dt = _dt;

  // wrench setpoint
  wrench_sp_subscriber =
      nh.subscribe("body_wrench_setpoint", 1, &FreeFloatingBodyPids::WrenchSPCallBack, this);
  // measure
  state_subscriber =
      nh.subscribe("state", 1, &FreeFloatingBodyPids::MeasureCallBack, this);

  // deal with controlled axes
  const size_t n = _controlled_axes.size();
  axes.resize(n);

  const std::vector<std::string> axes3D{"x", "y", "z", "roll", "pitch", "yaw"};

  // get whether or not we use dynamic reconfigure(reconfiguration)
  bool use_dynamic_reconfig;
  ros::NodeHandle control_node(nh, "controllers");
  //Assign value of use_dynamic_reconfig from server, with default(true).
  control_node.param("controllers/config/body/dynamic_reconfigure", use_dynamic_reconfig, true);

  if(n)
  {
    // position setpoint
    // std::cout << "I am before the position call back function" << std::endl;
    position_sp_subscriber =
        nh.subscribe("body_position_setpoint", 1, &FreeFloatingBodyPids::PositionSPCallBack, this);//topic,queue_size,callback,transport_hints(specify transport layer)
    // velocity setpoint(due to the timer)
    // ros::Timer timer = nh.createTimer(ros::Duration(10), &FreeFloatingBodyPids::TimerCallback );
    // std::cout << " I am in the loop" << std::endl;
    velocity_sp_subscriber =
        nh.subscribe("body_velocity_setpoint", 1, &FreeFloatingBodyPids::VelocitySPCallBack, this);

        // std::cout << "iter is \n" << iter++ << std::endl;

    body_states_subscriber = nh.subscribe("/vectored_auv/state", 1, &FreeFloatingBodyPids::BodyStateCallBack, this);

    effort_subscriber = nh.subscribe("/vectored_auv/thruster_command", 1, &FreeFloatingBodyPids::EffortCallBack, this);

    angle_subscriber = nh.subscribe("/vectored_auv/joint_states", 1, &FreeFloatingBodyPids::AngleCallBack, this);

  }
  // std::cout << "I am in the second loop" << std::endl;





  for(unsigned int i=0;i<n;++i)
  {
      //retrun the value of new type(<size_t>)
      //size_t is a type: that the range is the maximu in the object platform
      //auto: can make the value auto declare the type and the space
      //idx is the distance from begin to the current control axes
    const auto idx = static_cast<size_t>(std::distance(axes3D.begin(),// return the distance from first to last
                                                       std::find(axes3D.begin(),//find the value from frits to last
                                                                 axes3D.end(),
                                                                 _controlled_axes[i])));
    auto axis = &axes[i];
    axis->name = _controlled_axes[i];
    // here we have the controlled axis and the relative command
    switch(idx)
    {
    case 0:
      axis->position.error = &(pose_lin_error_.x());
      axis->velocity.error = &(velocity_lin_error_.x());
      axis->position.command = axis->velocity.command = &(wrench_command_.force.x);
      break;
    case 1:
      axis->position.error = &(pose_lin_error_.y());
      axis->velocity.error = &(velocity_lin_error_.y());
      axis->position.command = axis->velocity.command = &(wrench_command_.force.y);
      break;
    case 2:
      axis->position.error = &(pose_lin_error_.z());
      axis->velocity.error = &(velocity_lin_error_.z());
      axis->position.command = axis->velocity.command = &(wrench_command_.force.z);
      break;
    case 3:
      axis->position.error = &(pose_ang_error_.x());
      axis->velocity.error = &(velocity_ang_error_.x());
      axis->position.command = axis->velocity.command = &(wrench_command_.torque.x);
      break;
    case 4:
      axis->position.error = &(pose_ang_error_.y());
      axis->velocity.error = &(velocity_ang_error_.y());
      axis->position.command = axis->velocity.command = &(wrench_command_.torque.y);
      break;
    case 5:
      axis->position.error = &(pose_ang_error_.z());
      axis->velocity.error = &(velocity_ang_error_.z());
      axis->position.command = axis->velocity.command = &(wrench_command_.torque.z);
      break;
    }
    InitPID(axis->position.pid, ros::NodeHandle(control_node, axis->name + "/position"), use_dynamic_reconfig);
    InitPID(axis->velocity.pid, ros::NodeHandle(control_node, axis->name + "/velocity"), use_dynamic_reconfig);
  }

  // default control = position
  CTreq req;
  CTres res;
  // req.axes = {"z"};
  if(n)
    ToPositionControl(req, res);
  else
    ToEffortControl(req, res);
  if(default_mode == "velocity")
  {
    ToVelocityControl(req, res);
  }
  else if(default_mode == "depth")
  {
    req.axes = {"x", "y", "yaw"};
    ToVelocityControl(req, res);
  }
  else if(default_mode == "effort")
  {
    req.axes = {"x", "y", "yaw"};
    ToEffortControl(req, res);
  }
  else if (default_mode == "vectored")
  {
    // std::cout<<"In the loop" << std::endl;
    req.axes = {"x", "y", "roll", "pitch", "yaw"};
    // req.axes = {"x"};
    ToVelocityControl(req, res);
    // req.axes = {"x"};
    // ToEffortControl(req, res);
    // std::cout<<"after to velocity control" << std::endl;
    // req.axes = {"z"};
    // ToPositionControl(req, res);
    // std::cout<<"after to position control" << std::endl;
  }
  initSwitchServices(control_node, "body");

  // plt::named_plot("v_x",v_x);
  // plt::named_plot("v_x setpoint"s_v_x);



}


//update the parameter:pose_ang_error_
//                     pose_lin_error_
//                     velocity_lin_error_
//                     velocity_ang_error_
//and return true after updating
bool FreeFloatingBodyPids::UpdatePID()
{
  bool updated = false;
  if(state_received)
  {
    if(setpoint_position_ok && position_used)    // setpoint & need for position PID
    {
      Eigen::Matrix3d world_to_body = pose_ang_measure_inv_.toRotationMatrix();//from Quaternion to matrix
      // express pose error in the body frame
      pose_lin_error_ = world_to_body * (pose_lin_setpoint_ - pose_lin_measure_);//position - measure
      // quaternion error in the body frame

      Eigen::Quaterniond q(pose_ang_setpoint_ * pose_ang_measure_inv_);
      const double sin_theta_over_2 = sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z());//sin(theta/2)
      if(std::abs(sin_theta_over_2) < 1e-6)
        pose_ang_error_ = Eigen::Vector3d(0,0,0);
      else
      {
        Eigen::Vector3d u(q.x(),q.y(),q.z());
        u *= 1./sin_theta_over_2;
        pose_ang_error_ = 2*atan2(sin_theta_over_2, q.w()) * world_to_body * u;//Q:how does it come from?
      }
      UpdatePositionPID();
      updated = true;
    }

    if(setpoint_velocity_ok && velocity_used)
    {
      // velocity error is already in the body frame - this is the setpoint error
      velocity_lin_error_ =  velocity_lin_setpoint_ - velocity_lin_measure_;
      velocity_ang_error_ =  velocity_ang_setpoint_ - velocity_ang_measure_;

      //cout << "Velocity lin error in WF: " << (velocity_lin_error_).transpose() << endl;
      // writes the wrench command
      UpdateVelocityPID();
      updated = true;
    }
  }
  return updated;
}


// parse received position setpoint
void FreeFloatingBodyPids::PositionSPCallBack(const geometry_msgs::PoseStampedConstPtr& _msg)
{
  // std::cout << " I am in the position callback function" << std::endl;
  setpoint_position_ok = true;
  double z = -2.0;
  float roll_setpoint, pitch_setpoint, yaw_setpoint;
  pitch_setpoint = yaw_setpoint = 0.0;

  roll_setpoint = 0;
  if((ros::Time::now().toSec()>= 10.0 && ros::Time::now().toSec()<=30.0))
    roll_setpoint = 3.14/6.0;
  else if((ros::Time::now().toSec()>=40.0 && ros::Time::now().toSec()<=60.0))
    roll_setpoint = 3.14/6.0;

  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(roll_setpoint, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(pitch_setpoint, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(yaw_setpoint, Eigen::Vector3f::UnitZ());

  // std::cout << "rotation matrix is " << q.toRotationMatrix() << std::endl;

  Eigen::Matrix3f rotation_Matrix;
  rotation_Matrix = Eigen::AngleAxisf(roll_setpoint, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(pitch_setpoint, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(yaw_setpoint, Eigen::Vector3f::UnitZ());

  Eigen::Vector3f rpy = rotation_Matrix.eulerAngles(0,1,2);

  // s_roll.push_back(rpy[0]);

  // std::cout << " angles are \n" << rpy << std::endl;


  // pose_lin_setpoint_ = Eigen::Vector3d(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
  pose_lin_setpoint_ = Eigen::Vector3d(_msg->pose.position.x, 0, z);
  pose_ang_setpoint_ = Eigen::Quaterniond(q);

  // pose_ang_setpoint_ = Eigen::Quaterniond(_msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z);
  // pose_ang_setpoint_ = Eigen::Quaterniond(_msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z);
  // plot the position of z
//  s_p_z.push_back(z);
  time_p_sp.push_back(ros::Time::now().toSec());

}

// parse received velocity setpoint
// void TimerCallback(ros::NodeHandle &nh)
// {
//   ros::Subscriber  velocity_sp_subscriber;
//   velocity_sp_subscriber =
//       nh.subscribe("body_velocity_setpoint", 1, &FreeFloatingBodyPids::VelocitySPCallBack, this);
// }

// void FreeFloatingBodyPids::VelocitySPCallBack(const geometry_msgs::TwistStampedConstPtr & _msg)
// {
//   setpoint_velocity_ok = true;
//   velocity_lin_setpoint_ = Eigen::Vector3d(_msg->twist.linear.x, _msg->twist.linear.y, _msg->twist.linear.z);
//   velocity_ang_setpoint_ = Eigen::Vector3d(_msg->twist.angular.x, _msg->twist.angular.y, _msg->twist.angular.z);
// }

void FreeFloatingBodyPids::EffortCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
  // std::vector<double> plot_effort_fl, plot_effort_fr, plot_angle_l, plot_angle_r;
  // std::vector<double> time_effort, time_angle;
  time_effort.push_back(ros::Time::now().toSec());
  plot_effort_fl.push_back(_msg->effort[0]);
  plot_effort_fr.push_back(_msg->effort[1]);
  plot_effort_f2.push_back(_msg->effort[2]);
  plot_effort_f3.push_back(_msg->effort[3]);
  plot_effort_f4.push_back(_msg->effort[4]);
}

void FreeFloatingBodyPids::AngleCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
  // std::vector<double> plot_effort_fl, plot_effort_fr, plot_angle_l, plot_angle_r;
  // std::vector<double> time_effort, time_angle;
  time_angle.push_back(ros::Time::now().toSec());
  plot_angle_l.push_back(_msg->position[0]);
  plot_angle_r.push_back(_msg->position[1]);
}


void FreeFloatingBodyPids::BodyStateCallBack(const nav_msgs::OdometryConstPtr &_msg)
{

  // plot the state
  v_x.push_back(_msg->twist.twist.linear.x);
  // v_y.push_back(_msg->twist.twist.linear.y);
  cur_depth = _msg->pose.pose.position.z;
  p_z.push_back(_msg->pose.pose.position.z);
  // roll.push_back(_msg->twist.twist.angular.x);
  float x ,y, z, w;
  x = _msg->pose.pose.orientation.x;
  y = _msg->pose.pose.orientation.y;
  z = _msg->pose.pose.orientation.x;
  w = _msg->pose.pose.orientation.x;
  Eigen::Matrix3f rotation_Matrix;
  Eigen::Quaternionf q;
  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;

  Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
  roll.push_back(_msg->twist.twist.angular.x);
  cur_roll = rpy[0];
  if (cur_roll >= 3.0)
    cur_roll = cur_roll - 3.14159;
//  std::cout << "current roll is : " << cur_roll << std:: endl;
  // pitch.push_back(_msg->twist.twist.angular.y);
  // yaw.push_back(_msg->twist.twist.angular.z);

  time_state.push_back(ros::Time::now().toSec());


}

void FreeFloatingBodyPids::VelocitySPCallBack(const geometry_msgs::TwistStampedConstPtr & _msg)
{
  // velocity setfunction
  setpoint_velocity_ok = true;
  ros::Time t = ros::Time::now();
  double z = -3.0;
  s_p_z.push_back(z);
  double vy = 0;
  double vz = 0.1 * (z - cur_depth);
  double vx = 0.0;

//  if(ros::Time::now().toSec()>=10 && ros::Time::now().toSec() <= 70)
//    x = 0.05;


// /vectored_auv/state/twist/twist/linear/x
// /vectored_auv/state/twist/twist/angular/x


  double desird_roll = 0.5;
//  double v_roll = 0.1 * (desird_roll - cur_roll) ;
  double v_roll = 0.1;

  double v_pitch = 0.0;
  double v_yaw = 0.0;
//  roll.push_back(desird_roll);


    velocity_lin_setpoint_ = Eigen::Vector3d(vx, vy, vz);
    velocity_ang_setpoint_ = Eigen::Vector3d(v_roll, v_pitch, v_yaw);

  s_v_x.push_back(vx);
  // s_v_y.push_back(y);
  // s_v_z.push_back(-0.02);
  s_roll.push_back(v_roll);
  // s_pitch.push_back(v_pitch);
  // s_yaw.push_back(0);

  time_sp.push_back(ros::Time::now().toSec());
  // to output the time
  std::cout << ros::Time::now().toSec() << std::endl;

  if(ros::Time::now().toSec()>= 30&& ros::Time::now().toSec()<=30.2)
  {
    //to plot the x and roll velocity
  plt::figure();
  plt::named_plot("actual velocity",time_state,v_x);
  plt::named_plot("desired velocity",time_sp,s_v_x);
  plt::legend();
  plt::title("vx");
  plt::save("/home/x1ao/master/master_thesis_auv/test01/vx.png");

  plt::figure();
  plt::named_plot("actual roll",time_state,roll);
  plt::named_plot("desired roll",time_sp,s_roll);
  plt::legend();
  plt::title("roll");
  plt::save("/home/x1ao/master/master_thesis_auv/test01/roll.png");

  plt::figure();
  plt::named_plot("actual depth",time_state,p_z);
  plt::named_plot("desired depth",time_sp,s_p_z);
  plt::legend();
  plt::title("depth");
  plt::save("/home/x1ao/master/master_thesis_auv/test01/depth.png");

  plt::figure();
  plt::named_plot("left",time_effort,plot_effort_fl);
  plt::named_plot("right",time_effort,plot_effort_fr);
  // plt::ylim(-35, 35);
  plt::legend();
  plt::title("thruster force");
  plt::save("/home/x1ao/master/master_thesis_auv/test01/vforce.png");

  plt::figure();
  plt::named_plot("left",time_angle,plot_angle_l);
  plt::named_plot("right",time_angle,plot_angle_r);
  plt::legend();
  plt::title("thruster angle");
  plt::save("/home/x1ao/master/master_thesis_auv/test01/angles.png");


  plt::figure();
  plt::named_plot("th2",time_effort,plot_effort_f2);
  plt::named_plot("th3",time_effort,plot_effort_f3);
  plt::named_plot("th4",time_effort,plot_effort_f4);
  // plt::ylim(-35, 35);
  plt::legend();
  plt::title("th2 to th4");
  plt::save("/home/x1ao/master/master_thesis_auv/test01/force.png");



  // plt::figure();
  // plt::named_plot("actual velocity",time_state,v_y);
  // plt::named_plot("desired velocity",time_sp,s_v_y);
  // plt::legend();
  // plt::title("vy");
  // plt::save("/home/x1ao/master/master_thesis_auv/test01/pure_velocity_tracking_vy.svg");
  //
  // plt::figure();
  // plt::named_plot("actual velocity",time_state,v_z);
  // plt::named_plot("desired velocity",time_sp,s_v_z);
  // plt::legend();
  // plt::title("vz");
  // plt::save("/home/x1ao/master/master_thesis_auv/test01/pure_velocity_tracking_vz.svg");

  // plt::figure();
  // plt::named_plot("actual velocity",time_state,pitch);
  // plt::named_plot("desired velocity",time_sp,s_pitch);
  // plt::legend();
  // plt::title("pitch");
  // plt::save("/home/x1ao/master/master_thesis_auv/test01/pure_velocity_tracking_pitch.svg");
  //
  // plt::figure();
  // plt::named_plot("actual velocity",time_state,yaw);
  // plt::named_plot("desired velocity",time_sp,s_yaw);
  // plt::legend();
  // plt::title("yaw");
  // plt::save("/home/x1ao/master/master_thesis_auv/test01/pure_velocity_tracking_yaw.svg");

  std::cout << "already ploted" << std::endl;
  }




  // std::cout << "iter in setpoint is \n" << iter++ << std::endl;

}

void FreeFloatingBodyPids::MeasureCallBack(const nav_msgs::OdometryConstPtr &_msg)
{

  // std::cout << "iter in state call back is \n" << iter_state++ << std::endl;


  state_received = true;
  // positions are expressed in the world frame, rotation is inversed(from measure function)
  pose_lin_measure_ = Eigen::Vector3d(_msg->pose.pose.position.x, _msg->pose.pose.position.y, _msg->pose.pose.position.z);
  pose_ang_measure_inv_ = Eigen::Quaterniond(_msg->pose.pose.orientation.w, _msg->pose.pose.orientation.x, _msg->pose.pose.orientation.y, _msg->pose.pose.orientation.z).inverse();

  // change velocities from world to body frame
  velocity_lin_measure_ = pose_ang_measure_inv_.toRotationMatrix()*Eigen::Vector3d(_msg->twist.twist.linear.x, _msg->twist.twist.linear.y, _msg->twist.twist.linear.z);
  velocity_ang_measure_ = pose_ang_measure_inv_.toRotationMatrix()*Eigen::Vector3d(_msg->twist.twist.angular.x, _msg->twist.twist.angular.y, _msg->twist.twist.angular.z);


}

