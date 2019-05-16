# freefloating
The master thesis: Control of an Autunomous Underwater with Vectored Thrusters

/freefloating_gazebo : is to do the PID control and control allocation in gazebo
/freefloating_gazebo_demo : is the vectored thrusters model and the environment
/slider_publisher : is to publish the desired velocity/position or effort by slider

The plots are not in this folder and tha main problem now is the roll motion are not following the desired one.

Try to find the caurse of errors
PID gains:
Linear velocity in x axis(vx) :
with desired angular velocity in x axis is always 0 and with linear velocity in x axis:
At the beginnign, the actual linear velocity gose to the desired one and the angular velocity is equal to 0 but at the time: the angular velocity has the big errors

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/vx.png)


![Image text](https://github.com/x1aoo/freefloating/raw/master/image/roll.png)
