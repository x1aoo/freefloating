#include <freefloating_gazebo/thruster_allocator.h>
#include <visp/vpFeaturePoint.h>
// #include <freefloating_gazebo/optim.h>
#include <algorithm>
#include <ecn_common/vpQuadProg.h>

#include </home/x1ao/master/master_thesis_auv/ros_auv/src/freefloating_gazebo/src/matplotlibcpp.h>

namespace plt = matplotlibcpp;

namespace ffg
{

// parse from robot_description param
ThrusterAllocator::ThrusterAllocator(ros::NodeHandle &nh)
{
  HydroModelParser parser;
  parser.parseAll(nh);
  // store link
  base_link = parser.getLinks().find("base_link")->second;

  // keep dynamics
  parser.thrusterInfo(fixed_idx, steer_idx, names, max_thrust);
  max_wrench = parser.maxWrench();//find the details from the robot_description
  max_wrench[0] = 60;
  max_wrench[3] = 27;
  max_wrench[5] = 5;
  // std::cout << "-----------------------------------------------" << std::endl;
  // std::cout << "max_wrench in x is " << max_wrench[0] << std::endl;
  // std::cout << "max_wrench in y is " << max_wrench[1] << std::endl;
  // std::cout << "max_wrench in z is " << max_wrench[2] << std::endl;
  // std::cout << "max_wrench in roll is " << max_wrench[3] << std::endl;
  // std::cout << "max_wrench in pitch is " << max_wrench[4] << std::endl;
  // std::cout << "max_wrench in yaw is " << max_wrench[5] << std::endl;
  // std::cout << "-----------------------------------------------" << std::endl;
  max_vel = parser.maxVelocity();
  map = parser.thrusterMap();
}

//to push the controlled axes to the nh and map_threshold is useless
std::vector<std::string> ThrusterAllocator::initControl(ros::NodeHandle &nh, double map_threshold)
{
  std::vector<std::string> controlled_axes;
  const std::vector<std::string> axes{"x", "y", "z", "roll", "pitch", "yaw"};
  for(size_t axis = 0; axis < 6; axis++)
  {
    if(max_wrench[axis] > 1e-6)//come from parsing from robot_description param,when it is not the noise
    {
      // std::cout<<"max_wrench " << axis << " is " << max_wrench[axis] << std::endl;
      char param[FILENAME_MAX];//what is [FILENAME_MAX]?
      sprintf(param, "controllers/%s", axes[axis].c_str());//axes[axis].c_str() return the desired axes
      std::cout << param << std::endl;
      if(nh.hasParam(param))
      {
        controlled_axes.push_back(axes[axis]);//controlled_axes: the axes which would be changed
        std::cout << "controlled axes are: ";
        for (auto i = controlled_axes.begin(); i != controlled_axes.end(); ++i)
          std::cout << *i << ' ';
        std::cout << std::endl;
        // push to position PID
        sprintf(param, "controllers/%s/position/i_clamp", axes[axis].c_str());
        nh.setParam(param, max_wrench[axis]);//Set a string value on the parameter server.
        // push to velocity PID
        sprintf(param, "controllers/%s/velocity/i_clamp", axes[axis].c_str());
        nh.setParam(param, max_wrench[axis]);
      }
    }
  }

  // threshold on map before pseudo-inverse
  for(int r = 0; r < 6; ++r)
  {
    for(int c = 0; c < map.cols(); ++c)
    {
      if(std::abs(map(r,c)) < map_threshold)
        map(r,c) = 0;
    }
  }

  inverse_map.resize(map.cols(), map.rows());
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_M(map, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd dummy_in(map.rows());
  Eigen::VectorXd dummy_out(map.cols());
  unsigned int i,j;
  for(i=0;i<map.rows();++i)
  {
    dummy_in.setZero();
    dummy_in(i) = 1;
    dummy_out = svd_M.solve(dummy_in);
    for(j = 0; j<map.cols();++j)
      inverse_map(j,i) = dummy_out(j);
  }
  return controlled_axes;
}

void ThrusterAllocator::saturate(Eigen::VectorXd &_command) const
{
  double norm_ratio = 1;
  unsigned int i;
  for(i=0;i<fixed_idx.size();++i)
    norm_ratio = std::max(norm_ratio, std::abs(_command(i)) / max_thrust[i]);
  _command *= 1./norm_ratio;
}

sensor_msgs::JointState ThrusterAllocator::wrench2Thrusters(const geometry_msgs::Wrench &cmd, double angle1, double angle2, double fl, double fr, double f2, double f3, double f4, ros::NodeHandle &nh) const
{

  double lx1,lx2,lx4,lx5,ly1,ly2;
  lx1 = lx2 = -1.5/3;
  ly1 = 0.13 + 0.05;
  ly2 = -ly1;
  lx4 = -0.3*1.5;
  lx5 = -lx4;

  double f_max, f_min, alphamax, alphamin, deltalphamax, deltalphamin;
  f_max = 30.0;
  f_min = -30.0;
  alphamax = 3.14/2;
  alphamin = -3.14/2;
  deltalphamax = 5.0/180.0*3.14;
  deltalphamin = -5.0/180.0*3.14;
//  deltalphamax = 5.0;
//  deltalphamin = -5.0;



  vpMatrix W(7,7), H(7,7), A(14,7), Aeq(6,7), Aeq_det(6,6);
  vpColVector f(7), b(14), beq(6), state_pre(7), state(7), delta_state(7), tau(6);
  vpRowVector Q(7);
  state_pre[0] = fl;
  state_pre[1] = fr;
  state_pre[2] = f2;
  state_pre[3] = f3;
  state_pre[4] = f4;
  state_pre[5] = angle1;
  state_pre[6] = angle2;

  W[5][5] = W[6][6] = 0;
  W[0][0] = W[1][1] =  1;
  W[2][2] = W[3][3] = W[4][4] = 1;
  H = 2*(W.transpose()*W);

  A[0][0] = A[2][1] = A[4][2] = A[6][3] = A[8][4] = 1;
  A[10][5] = A[12][6] = 1;
  A[1][0] = A[3][1] = A[5][2] = A[7][3] = A[9][4] = -1;
  A[11][5] = A[13][6] = -1;

  tau[0] = cmd.force.x;
  tau[1] = cmd.force.y;
  tau[2] = cmd.force.z;
  tau[3] = cmd.torque.x;
  tau[4] = cmd.torque.y;
  tau[5] = cmd.torque.z;
  // std::cout << "the tau is \n" << tau << std::endl;

    Aeq[0][0] = cos(state_pre[5]);
    Aeq[2][0] = -sin(state_pre[5]);
    Aeq[3][0] = -sin(state_pre[5]) * ly1;
    Aeq[4][0] = sin(state_pre[5]) * lx1;
    Aeq[5][0] = -cos(state_pre[5]) * ly1;

    Aeq[0][1] = cos(state_pre[6]);
    Aeq[2][1] = -sin(state_pre[6]);
    Aeq[3][1] = -sin(state_pre[6]) * ly2;
    Aeq[4][1] = sin(state_pre[6]) * lx2;
    Aeq[5][1] = -cos(state_pre[6]) * ly2;

    Aeq[1][2] = 1;

    Aeq[2][3] = -1;
    Aeq[4][3] = lx4;

    Aeq[2][4] = -1;
    Aeq[4][4] = lx5;

    beq = tau;


    b[0] = f_max ;
    b[1] =  -f_min;
    b[2] = f_max;
    b[3] =  -f_min;
    b[4] = f_max;
    b[5] =  -f_min;
    b[6] = f_max ;
    b[7] =  -f_min;
    b[8] = f_max;
    b[9] = -f_min;
    b[10] = std::min(deltalphamax, alphamax - state_pre[5]);
    b[11] = std::min(-deltalphamin, state_pre[5] - alphamin);
    b[12] = std::min(deltalphamax, alphamax - state_pre[6]);
    b[13] = std::min(-deltalphamin, state_pre[6] - alphamin);

    vpQuadProg qp;
    qp.vpQuadProg::solveQP(H,f,Aeq,beq,A,b,delta_state);

    // std::cout << "delta state of angle1 is = " << delta_state[5] << std::endl;

  state[0] = delta_state[0];
  state[1] = delta_state[1];
  state[2] = delta_state[2];
  state[3] = delta_state[3];
  state[4] = delta_state[4];
  state[5] = state_pre[5] + delta_state[5];
  state[6] = state_pre[6] + delta_state[6];
  // state = state_pre + delta_state;
  state_pre = state;
  // std::cout << "the equation function is \n" << beq - Aeq * state << std::endl;
  // std::cout << "state is \n" << state << std::endl;

  sensor_msgs::JointState msg;
  msg.name = names;
  msg.effort.reserve(names.size());

  for(int i = 0; i < state.size()-2; i++)
    msg.effort.push_back(state[i]);

  sensor_msgs::JointState joint_msg;

  joint_msg.name.push_back("fwd_left");
  joint_msg.name.push_back("fwd_right");
  joint_msg.position.push_back(state[5]);
  joint_msg.position.push_back(state[6]);

  ros::Publisher Joint_Command_Publisher;
  Joint_Command_Publisher = nh.advertise<sensor_msgs::JointState>("/vectored_auv/joint_setpoint", 1);
  Joint_Command_Publisher.publish(joint_msg);


  return msg;

}



sensor_msgs::JointState ThrusterAllocator::wrench2Thrusters_iterative(const geometry_msgs::Wrench &cmd, vpColVector state_pre, ros::NodeHandle &nh) const
{

  // std::cout << "state_pre\n" << state_pre << std::endl;
  double body_length, body_radius, tr2, tr1, tl1;
  body_length = 1.5;
  body_radius = 0.13;
  tr2 = .05;
  tr1 = 0.02;
  tl1 = 0.1;
  double lx1,lx2,lx4,lx5,ly1,ly2;

  lx1 = lx2 = -body_length/3.0;
  ly1 = body_radius+tr2;
  ly2 = -(body_radius+tr2);
  lx4 = -0.3*body_length;
  lx5 = 0.3*body_length;

  double f_max, f_min, alphamax, alphamin, deltalphamax, deltalphamin;
  f_max = 30.0;
  f_min = -30.0;
  alphamax = 3.14/2.0;
  alphamin = -3.14/2.0;
  deltalphamax = 5.0/180.0*3.14;
  deltalphamin = -5.0/180.0*3.14;
//   deltalphamin = -5.0;
//    deltalphamax = 5.0;

    vpMatrix W(7,7), A(14,7), Aeq_det(6,6), H(1,7);
  vpColVector f(7), state_initial(7), state(7), tau(6), r(1) ;
  vpRowVector Q(7);
  vpMatrix Aeq_pre(6,7), Aeq(6,7);
  vpColVector b(14), beq(6), delta_state(7), cost(6), cost_pre(6) ;

  double cost_delta = 0.0;
  delta_state[0] = 10;

  W[5][5] = W[6][6] = 2;
  W[0][0] = W[1][1] = 1;
  W[2][2] = W[3][3] = W[4][4] = 1;

  A[0][0] = A[2][1] = A[4][2] = A[6][3] = A[8][4] = 1.0;
  A[10][5] = A[12][6] = 1.0;
  A[1][0] = A[3][1] = A[5][2] = A[7][3] = A[9][4] = -1.0;
  A[11][5] = A[13][6] = -1.0;

  tau[0] = cmd.force.x;
  tau[1] = cmd.force.y;
  tau[2] = cmd.force.z;
  tau[3] = cmd.torque.x;
  tau[4] = cmd.torque.y;
  tau[5] = cmd.torque.z;
  int iter = 0;
  double eps = 0.01;
//
  std::vector<double> vec_delta_state_fl, vec_state_fl;
  std::vector<double> vec_iter;
  bool in_loop = true;
  while(iter<=1000 && in_loop)
  {
  //   // vpMatrix Aeq_pre(6,7), Aeq(6,7);
    // vpColVector b(14), beq(6), delta_state(7);

    vec_iter.push_back(double(iter++));
    std::cout << "iter is " << iter << std::endl;

    vec_delta_state_fl.push_back(cost_delta);
    vec_state_fl.push_back(state[0]);
    // // std::cout << "iter is " << iter << std::endl;
    if(iter == 1000)
    {
      std::cout << "iter is " << iter << "\n" << "the maximun value is \n"
      << delta_state.getMaxValue() << "\n" << delta_state.getMinValue() << std::endl;

      // plt::figure();
      // plt::named_plot("state",vec_iter,vec_state_fl);
      // plt::legend();
      // plt::title("state in iter 1000");
      // plt::save("/home/x1ao/master/master_thesis_auv/test01/state_changed_1000.png");

      plt::figure();
      plt::named_plot("state",vec_iter,vec_delta_state_fl);
      plt::legend();
      plt::title("delta state in iter 1000");
      plt::save("/home/x1ao/master/master_thesis_auv/test01/delta_state_changed_1000.png");

    }
    // Aeq J(a,f)
    // std::cout << "angle1 is \n" << state_pre[5] << std::endl;
    Aeq[0][0] = cos(state_pre[5]);
    Aeq[2][0] = -sin(state_pre[5]);
    Aeq[3][0] = -sin(state_pre[5]) * ly1;
    Aeq[4][0] = sin(state_pre[5]) * lx1;
    Aeq[5][0] = -cos(state_pre[5]) * ly1;
    // std::cout << "cos(angle1) is " << Aeq[0][0] << std::endl;

    Aeq[0][1] = cos(state_pre[6]);
    Aeq[2][1] = -sin(state_pre[6]);
    Aeq[3][1] = -sin(state_pre[6]) * ly2;
    Aeq[4][1] = sin(state_pre[6]) * lx2;
    Aeq[5][1] = -cos(state_pre[6]) * ly2;

    Aeq[1][2] = 1.0;

    Aeq[2][3] = -1.0;
    Aeq[4][3] = lx4;

    Aeq[2][4] = -1.0;
    Aeq[4][4] = lx5;

    Aeq[0][5] = -sin(state_pre[5]) * state_pre[0];
    Aeq[2][5] = -cos(state_pre[5]) * state_pre[0];
    Aeq[3][5] = -cos(state_pre[5]) * state_pre[0] * ly1;
    Aeq[4][5] = cos(state_pre[5]) * state_pre[0] * lx1 ;
    Aeq[5][5] = sin(state_pre[5]) * state_pre[0] * ly1;

    Aeq[0][6] = -sin(state_pre[6]) * state_pre[1];
    Aeq[2][6] = -cos(state_pre[6]) * state_pre[1];
    Aeq[3][6] = cos(state_pre[6]) * state_pre[1] * ly2;
    Aeq[4][6] = cos(state_pre[6]) * state_pre[1] * lx2;
    Aeq[5][6] = sin(state_pre[6]) * state_pre[1] * ly2;

    // Aeq_pre - T(a)
    Aeq_pre[0][0] = Aeq[0][0];
    Aeq_pre[2][0] = Aeq[2][0];
    Aeq_pre[3][0] = Aeq[3][0];
    Aeq_pre[4][0] = Aeq[4][0];
    Aeq_pre[5][0] = Aeq[5][0];
    Aeq_pre[0][1] = Aeq[0][1];
    Aeq_pre[2][1] = Aeq[2][1];
    Aeq_pre[3][1] = Aeq[3][1];
    Aeq_pre[4][1] = Aeq[4][1];
    Aeq_pre[5][1] = Aeq[5][1];
    Aeq_pre[1][2] = Aeq[1][2];
    Aeq_pre[2][3] = Aeq[2][3];
    Aeq_pre[4][3] = Aeq[4][3];
    Aeq_pre[2][4] = Aeq[2][4];
    Aeq_pre[4][4] = Aeq[4][4];

    // state is delta f and delta alpha
    // beq is \Delta tau
    beq = tau - Aeq_pre * state_pre;

    //the bouyancy is compensate by PID control
    // beq[2] = beq[2] + (25 * 0.01)*9.8;

    // state is delta f delta alpha
    b[0] = f_max - state_pre[0];
    b[1] = state_pre[0]-f_min;
    b[2] = f_max- state_pre[1];
    b[3] = state_pre[1]-f_min;
    b[4] = f_max- state_pre[2];
    b[5] = state_pre[2]-f_min;
    b[6] = f_max- state_pre[3];
    b[7] = state_pre[3]-f_min;
    b[8] = f_max- state_pre[4];
    b[9] = state_pre[4]-f_min;
    b[10] = std::min(deltalphamax, alphamax - state_pre[5]);
    b[11] = std::min(-deltalphamin, state_pre[5] - alphamin);
    b[12] = std::min(deltalphamax, alphamax - state_pre[6]);
    b[13] = std::min(-deltalphamin, state_pre[6] - alphamin);

    vpQuadProg qp;
    qp.solveQPi(Aeq,beq, A,b,delta_state);

    cost = Aeq * delta_state - beq;
    cost_delta = cost.transpose() * cost - cost_pre.transpose() * cost_pre;
    cost_pre = cost;

    state = delta_state + state_pre;
    state_pre = state;

    if(cost_delta <= eps)
      in_loop = false;

  }

    cost = Aeq * delta_state - beq;
    double cost_sum = cost.transpose() * cost;



    plot_time.push_back(ros::Time::now().toSec());
    plot_cost_x.push_back(cost[0]);
    plot_cost_y.push_back(cost[1]);
    plot_cost_z.push_back(cost[2]);
    plot_cost_roll.push_back(cost[3]);
    plot_cost_roll_square.push_back(cost[3]*cost[3]);
    plot_cost_pitch.push_back(cost[4]);
    plot_cost_yaw.push_back(cost[5]);
    plot_cost_sum.push_back(cost_sum);
    plot_cost_rate.push_back((cost[3]*cost[3])/cost_sum);

    if(ros::Time::now().toSec()>= 142&& ros::Time::now().toSec()<=144.2)
    {
    plt::figure();
    plt::named_plot("tau in x",plot_time,plot_cost_x);
    plt::named_plot("tau in roll",plot_time,plot_cost_roll);
    plt::ylim(-100, 100);
    plt::legend();
    plt::title("torque in x and roll");
    plt::save("/home/x1ao/master/master_thesis_auv/test01/cost_rx.png");

    plt::figure();
    plt::named_plot("tau in y",plot_time,plot_cost_y);
    plt::named_plot("tau in roll",plot_time,plot_cost_roll);
    plt::ylim(-100, 100);
    plt::legend();
    plt::title("torque in y and roll");
    plt::save("/home/x1ao/master/master_thesis_auv/test01/cost_ry.png");

    plt::figure();
    // plt::named_plot("tau in x",plot_time,plot_cost_x);
    plt::named_plot("tau in z",plot_time,plot_cost_z);
    plt::named_plot("tau in roll",plot_time,plot_cost_roll);
    // plt::named_plot("tau in yaw",plot_time,plot_cost_yaw);
    plt::ylim(-100, 100);
    plt::legend();
    plt::title("torque in z and roll");
    plt::save("/home/x1ao/master/master_thesis_auv/test01/cost_rz.png");

    plt::figure();
    plt::named_plot("tau in pitch",plot_time,plot_cost_pitch);
    plt::named_plot("tau in roll",plot_time,plot_cost_roll);
    plt::ylim(-100, 100);
    plt::legend();
    plt::title("torque in pitch and roll");
    plt::save("/home/x1ao/master/master_thesis_auv/test01/cost_rp.png");

    plt::figure();
    // plt::named_plot("tau in x",plot_time,plot_cost_x);
    // plt::named_plot("tau in z",plot_time,plot_cost_z);
    plt::named_plot("tau in yaw",plot_time,plot_cost_yaw);
    plt::named_plot("tau in roll",plot_time,plot_cost_roll);

    plt::ylim(-100, 100);
    plt::legend();
    plt::title("torque in yaw and roll");
    plt::save("/home/x1ao/master/master_thesis_auv/test01/cost_ry.png");


    plt::figure();
    plt::named_plot("sum tau square",plot_time,plot_cost_sum);
    plt::named_plot("tau square in roll",plot_time,plot_cost_roll_square);

    plt::ylim(0.0, 1e5);
    plt::legend();
    plt::title("cost");
    plt::save("/home/x1ao/master/master_thesis_auv/test01/cost_rsum.png");

    plt::figure();
    plt::named_plot("cost of roll/sum of cost",plot_time,plot_cost_rate);
    // plt::named_plot("tau square in roll",plot_time,plot_cost_roll_square);

    // plt::ylim(0.0, 1e5);
    plt::legend();
    plt::title("cost rate");
    plt::save("/home/x1ao/master/master_thesis_auv/test01/cost_rsum_rate.png");
  }


    // std::cout << "Q is \n" << Aeq << "\n r is \n" << beq << "\n Aeq is \n" << Aeq_new << "\n beq is \n" << beq_new << "\n A is \n" << A << "\n b is \n" << b << std::endl;
    // std::cout << "cost is " << Aeq * delta_state - beq << std::endl;


//
  // }

// std::cout<<"iter = " << iter << std::endl;
  // std::cout << "the equation function is \n" << beq - Aeq * state << std::endl;

  // update angle

    sensor_msgs::JointState joint_msg;


    joint_msg.name.push_back("fwd_left");
    joint_msg.name.push_back("fwd_right");
    joint_msg.position.push_back(state[5]);
    joint_msg.position.push_back(state[6]);

    ros::Publisher Joint_Command_Publisher;
    Joint_Command_Publisher = nh.advertise<sensor_msgs::JointState>("/vectored_auv/joint_setpoint", 1);
    Joint_Command_Publisher.publish(joint_msg);

  sensor_msgs::JointState msg;
  msg.name = names;
  msg.effort.reserve(names.size());

  for(int i = 0; i < state.size()-2; i++)
    msg.effort.push_back(state[i]);

  return msg;
}

}
