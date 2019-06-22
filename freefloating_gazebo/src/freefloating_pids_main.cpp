#include <freefloating_gazebo/freefloating_pids_body.h>
#include <freefloating_gazebo/freefloating_pids_joint.h>
#include <freefloating_gazebo/thruster_allocator.h>
#include <memory>
#include <visp/vpFeaturePoint.h>
#include </home/x1ao/master/master_thesis_auv/ros_auv/src/freefloating_gazebo/src/matplotlibcpp.h>
//dynamic reconfigure
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/TutorialsConfig.h>
#include <Eigen/Dense>

using std::cout;
using std::endl;
vpColVector state_pre(7);
vpColVector state_posi(3);
vpColVector state_vel(8);
std::vector<double>gains;
//double kp1_gains, kp2_gains, kp3_gains, kw1_gains, kw2_gains, kw3_gains;
Eigen::Matrix3d curR;

double angle1, angle2, fl, fr, f2, f3, f4;
std::vector<double>v_x, v_y, v_z, roll, pitch, yaw;
// double test;
namespace plt = matplotlibcpp;
// iter = 0;


void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
//    ROS_INFO("Reconfigure Request: %d %d",
//             config.kp1,
//             config.kw1);
    std::vector<double> gains_inloop;
    gains_inloop.push_back(config.kp1);
//    double kp2 = 4 * sqrt(config.kp1);
//    double kp3 = 4 * sqrt(kp2);
    double kp2 = config.kp1 * config.kp1 / 3;
    double kp3 = sqrt(config.kp1*config.kp1*sqrt(kp3));
    gains_inloop.push_back(kp2);
    gains_inloop.push_back(kp3);
    gains_inloop.push_back(config.kw1);
    double kw2 = 4 * sqrt(config.kw1);
    double kw3 = 4 * sqrt(kw2);
    gains_inloop.push_back(kw2);
    gains_inloop.push_back(kw3);
    gains = gains_inloop;
}





void JointStateCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{

  // ROS_INFO("I am in the JointStateCallBack function");
  // std::cout << "I am in the JointStateCallBack function" << std::endl;
  angle1 = _msg->position[0];
  angle2 = _msg->position[1];
  state_pre[5] = angle1;
  state_pre[6] = angle2;
//  std::cout << "JointStateCallBack over" << std::endl;

}

 void BodyStateCallBack(const nav_msgs::OdometryConstPtr &_msg)
 {

     state_posi[0] = _msg->pose.pose.position.x;
     state_posi[1] = _msg->pose.pose.position.y;
     state_posi[2] = _msg->pose.pose.position.z;
     //std wrong here, should pass the rotation matrix directly
//     state_posi[3] = _msg->pose.pose.orientation.x/_msg->pose.pose.orientation.w;
//     state_posi[4] = _msg->pose.pose.orientation.y/_msg->pose.pose.orientation.w;
//     state_posi[5] = _msg->pose.pose.orientation.z/_msg->pose.pose.orientation.w;

     Eigen::Quaternion<double> curq(_msg->pose.pose.orientation.w,_msg->pose.pose.orientation.x,_msg->pose.pose.orientation.y,_msg->pose.pose.orientation.z);
     curR = curq.matrix();

//     std::cout << "curR is \n" << curR << std::endl;

     state_vel[0] = _msg->twist.twist.linear.x;
     state_vel[1] = _msg->twist.twist.linear.y;
     state_vel[2] = _msg->twist.twist.linear.z;
     state_vel[3] = _msg->twist.twist.angular.x;
     state_vel[4] = _msg->twist.twist.angular.y;
     state_vel[5] = _msg->twist.twist.angular.z;
//     std::cout << "BodyStateCallBack over" << std::endl;
 }


void ThrusterStateCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
  fl = _msg->effort[0];
  fr = _msg->effort[1];
  f2 = _msg->effort[2];
  f3 = _msg->effort[3];
  f4 = _msg->effort[4];
  state_pre[0] = fl;
  state_pre[1] = fr;
  state_pre[2] = f2;
  state_pre[3] = f3;
  state_pre[4] = f4;
//  std::cout << "ThrusterStateCallBack over" << std::endl;
}

int main(int argc, char ** argv)
{
  // init ROS node
  ros::init(argc, argv, "freefloating_pid_control");
  ros::NodeHandle nh;
  ros::NodeHandle control_node(nh, "controllers");//specify a namespace to the constructor, in the the rosservier would have a ../controlelrs/ as a name space
  ros::NodeHandle priv("~");

    dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
    dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");




  ffg::ThrusterAllocator allocator(nh);//create the NodeHandle named allocator and will parser all the information needed

  // wait for Gazebo running
  ros::service::waitForService("/gazebo/unpause_physics");
  const bool control_body = allocator.has_thrusters();
  const bool control_joints = FreeFloatingJointPids::writeJointLimits(nh);//not really understand

  // loop rate
  ros::Rate loop(100);
  ros::Duration dt(.01);

  ros::SubscribeOptions ops;

  // -- Init body ------------------
  // PID's class
  std::unique_ptr<FreeFloatingBodyPids> body_pid;
  ros::Publisher body_command_publisher, joint_setpoint_publisher;
  // to test the body_pid output
  // ros::Publisher body_pid_publisher;
  // std::string default_mode = "position";
   std::string default_mode = "velocity";
//  std::string default_mode = "vectored";

  std::stringstream ss;
  ss << "Init PID control for " << nh.getNamespace() << ": "; //Returns the namespace associated with this NodeHandle.
  //check if we need to control body
  if(control_body)
  {
    if(priv.hasParam("body_control"))//Check whether a parameter exists on the server
      priv.getParam("body_control", default_mode);//Resolves a name into full-qualified name and get a value from the server.

    const auto controlled_axes = allocator.initControl(nh, 0.07);//to push the controlled axes to the nh and the second parameter is useless
    body_pid = std::unique_ptr<FreeFloatingBodyPids>(new FreeFloatingBodyPids);//the unique pointer, which will release when go out its workspace
    body_pid->Init(nh, dt, controlled_axes, default_mode);

    // command
    body_command_publisher =
        nh.advertise<sensor_msgs::JointState>("thruster_command", 1);

    joint_setpoint_publisher =
        nh.advertise<sensor_msgs::JointState>("/vectored_auv/joint_setpoint", 1);


    // joint_publisher = nh.advertise<sensor_msgs::JointState>("joint_setpoint", 1);
    // to test the body_pid output
    // body_pid_publisher =
    //     nh.advertise<sensor_msgs::JointState>("wrench_command", 1);

    ss << controlled_axes.size() << " controlled axes (" << default_mode << " control)";
  }

  // -- Init joints ------------------
  std::unique_ptr<FreeFloatingJointPids> joint_pid;
  //check if we need to control joint,in our case, we need to control joints
  if(control_joints)
  {
    default_mode = "position";//the body control and the joint control may different
//      default_mode = "velocity";
    if(priv.hasParam("joint_control"))
      priv.getParam("joint_control", default_mode);

    joint_pid = std::unique_ptr<FreeFloatingJointPids>(new FreeFloatingJointPids);
    joint_pid->Init(nh, dt, default_mode);
  }

  std::vector<std::string> joint_names;
  if(control_joints)
  {
    control_node.getParam("config/joints/name", joint_names);//what the config stand for?because the parameter will change?
    ss << ", " << joint_names.size() << " joints (" << default_mode << " control)";
  }

  ROS_INFO("%s", ss.str().c_str());
  ros::Subscriber joint_states_subscriber, thruster_state_subscriber, body_states_subscriber;
  while(ros::ok())
  {
    // update body and publish
    if(control_body && body_pid->UpdatePID())
    {
    // std::cout << "body_pid is " << *body_pid << std::endl;
    // add the joint states in it.
        joint_states_subscriber = nh.subscribe("/vectored_auv/joint_states", 1, &JointStateCallBack);

         body_states_subscriber = nh.subscribe("/vectored_auv/state", 1, &BodyStateCallBack);
        // std::cout << "ros::Subscriber" << std::endl;

        thruster_state_subscriber = nh.subscribe("/vectored_auv/thruster_command", 1, &ThrusterStateCallBack);


        // body_command_publisher.publish(allocator.wrench2Thrusters(body_pid->WrenchCommand(), angle1, angle2, fl, fr, f2, f3, f4, nh));
        // if((int)(ros::Time::now().toSec())%2==0)
        
        body_command_publisher.publish(allocator.wrench2Thrusters_iterative(body_pid->WrenchCommand(), state_pre, nh));
//        body_command_publisher.publish(allocator.wrench2Thrusters_3rd(body_pid->WrenchCommand(), state_pre, state_posi, state_vel, nh, gains, curR));

        // to test the body_pid output
        // body_pid_publisher.publish(body_pid->WrenchCommand());
        // cout<<"body_pid->WrenchCommand() = "<<body_pid->WrenchCommand()<<endl;
    }
    // update joints and publish
    if(control_joints && joint_pid->UpdatePID())
      joint_pid->publish();

    ros::spinOnce();
    loop.sleep();



  }
  // plt::plot(v_x);
  // plt::save("/home/x1ao/master/master_thesis_auv/plot_pure_velocity_tracking/pure_velocity_tracking_01.png");
}
