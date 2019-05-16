#include <freefloating_gazebo/freefloating_pids_body.h>
#include <freefloating_gazebo/freefloating_pids_joint.h>
#include <freefloating_gazebo/thruster_allocator.h>
#include <memory>
#include <visp/vpFeaturePoint.h>
#include </home/x1ao/master/master_thesis_auv/ros_auv/src/freefloating_gazebo/src/matplotlibcpp.h>

using std::cout;
using std::endl;
vpColVector state_pre(7);
double angle1, angle2, fl, fr, f2, f3, f4;
std::vector<double>v_x, v_y, v_z, roll, pitch, yaw;
// double test;
namespace plt = matplotlibcpp;
// iter = 0;

void JointStateCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{

  // ROS_INFO("I am in the JointStateCallBack function");
  // std::cout << "I am in the JointStateCallBack function" << std::endl;
  angle1 = _msg->position[0];
  angle2 = _msg->position[1];
  state_pre[5] = angle1;
  state_pre[6] = angle2;
  // std::cout << "angle 1 = \n" << angle1 << std::endl;
  // v_x.push_back(_msg->velocity[0]);
  // v_y.push_back(_msg->velocity[1]);
  // v_z.push_back(_msg->velocity[2]);
  // roll.push_back(_msg->velocity[3]);
  // pitch.push_back(_msg->velocity[4]);
  // yaw.push_back(_msg->velocity[5]);
  // std::cout << "v_x = " << v_x <<  std::endl;
}

// void BodyStateCallBack(const nav_msgs::OdometryConstPtr &_msg)
// {
//
//   // ROS_INFO("I am in the JointStateCallBack function");
//   // std::cout << "I am in the JointStateCallBack function" << std::endl;
//   // test = _msg->pose.pose.position.x;
//   // angle2 = _msg->pose.pose.position.y;
//   // state_pre[5] = angle1;
//   // state_pre[6] = angle2;
//   v_x.push_back(_msg->twist.twist.linear.x);
//   v_y.push_back(_msg->twist.twist.linear.y);
//   v_z.push_back(_msg->twist.twist.linear.z);
//   roll.push_back(_msg->twist.twist.angular.x);
//   pitch.push_back(_msg->twist.twist.angular.y);
//   yaw.push_back(_msg->twist.twist.angular.z);
//   // std::cout << "test = \n" << test << std::endl;
//
//
//   // std::cout << "v_x = " << v_x <<  std::endl;
// }


void ThrusterStateCallBack(const sensor_msgs::JointStateConstPtr &_msg)
{
  // ROS_INFO("I am in the JointStateCallBack function");
  // std::cout << "I am in the ThrusterStateCallBack function" << std::endl;

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
  // std::cout << "f is \n" << fl << fr << f2 << f3 << f4 << std::endl;
  // std::cout << "angle1 = " << angle1 << " angle2 = " << angle2 << std::endl;
}

int main(int argc, char ** argv)
{
  // init ROS node
  ros::init(argc, argv, "freefloating_pid_control");
  ros::NodeHandle nh;
  ros::NodeHandle control_node(nh, "controllers");//specify a namespace to the constructor, in the the rosservier would have a ../controlelrs/ as a name space
  ros::NodeHandle priv("~");

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
  ros::Publisher body_command_publisher;
  // to test the body_pid output
  // ros::Publisher body_pid_publisher;
  // std::string default_mode = "position";
  // std::string default_mode = "velocity";
  std::string default_mode = "vectored";

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

        // body_states_subscriber = nh.subscribe("/vectored_auv/state", 1, &BodyStateCallBack);
        // std::cout << "ros::Subscriber" << std::endl;

        thruster_state_subscriber = nh.subscribe("/vectored_auv/thruster_command", 1, &ThrusterStateCallBack);


        // body_command_publisher.publish(allocator.wrench2Thrusters(body_pid->WrenchCommand(), angle1, angle2, fl, fr, f2, f3, f4, nh));
        // if((int)(ros::Time::now().toSec())%2==0)
        
          body_command_publisher.publish(allocator.wrench2Thrusters_iterative(body_pid->WrenchCommand(), state_pre, nh));


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