//calculate the torque by the error of velocity and than use the control allocation to do the allocate
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_srvs/Empty.h>

using std::cout;
using std::endl;

int main(int argc, char ** argv)
{
    // init ROS node
    ros::init(argc, argv, "freefloating_pid_control");
    // ros::NodeHandle nh;
    ros::NodeHandle ros_node;
    ros::NodeHandle priv("~");
    const double rate = 30;
    ros::Rate loop(rate);


//    ros::Publisher
    // water current
    geometry_msgs::Vector3 current;
    current.y = .1;
    // current.z= -25;
    ros::Publisher current_publisher = ros_node.advertise<geometry_msgs::Vector3>("/gazebo/current", 1);
    // ensure physics are running
    std_srvs::Empty srv;
    ros::ServiceClient client = ros_node.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    cout << "Unpausing physics" << endl;
    client.call(srv);

    while(ros::ok())
    {
        // update body and publish
        current_publisher.publish(current);



        ros::spinOnce();
        loop.sleep();
    }
}
