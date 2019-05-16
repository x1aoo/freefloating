# freefloating
The master thesis: Control of an Autunomous Underwater with Vectored Thrusters

/freefloating_gazebo : is to do the PID control and control allocation in gazebo
/freefloating_gazebo_demo : is the vectored thrusters model and the environment
/slider_publisher : is to publish the desired velocity/position or effort by slider

The plots are not in this folder and tha main problem now is the roll motion are not following the desired one.

Try to find the caurse of errors

Linear velocity in x axis(vx) :
with desired angular velocity in x axis is always 0 and with linear velocity in x axis:

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/vx.png)


![Image text](https://github.com/x1aoo/freefloating/raw/master/image/roll.png)

At the beginning, the actual linear velocity gose to the desired one and the angular velocity is equal to 0 but at the time:50 the angular velocity has the big errors and at the time 55 the linear velocity has the errors

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/angles.png)

Since we do not set the desired angular velocity in x axis, the angles are not expected to have the big change like that but it may because the robot want to stablize the angular velocity(roll) but from the roll plot it do not have the position effect. It seems that the algorithm can not control the robot in a proper way. 

However, I already checked several times the optimization algorithm(QP) and do not find the errors. But may be due to the fact of PID gains of angular velcocity in x axis(roll).
