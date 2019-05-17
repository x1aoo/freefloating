# freefloating
The master thesis: Control of an Autunomous Underwater with Vectored Thrusters

/freefloating_gazebo : is to do the PID control and control allocation in gazebo
/freefloating_gazebo_demo : is the vectored thrusters model and the environment
/slider_publisher : is to publish the desired velocity/position or effort by slider

The plots are not in this folder and tha main problem now is the roll motion are not following the desired one.

## Debug report:

### Try to find the caurse of errors

Linear velocity in x axis(vx) :
with desired angular velocity in x axis is always 0 and with linear velocity in x axis:

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/vx.png)


![Image text](https://github.com/x1aoo/freefloating/raw/master/image/roll.png)

At the beginning, the actual linear velocity gose to the desired one and the angular velocity is equal to 0 but at the time:50 the angular velocity has the big errors and at the time 55 the linear velocity has the errors

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/angles.png)

Since we do not set the desired angular velocity in x axis, the angles are not expected to have the big change like that but it may because the robot want to stablize the angular velocity(roll) but from the roll plot it do not have the position effect. It seems that the algorithm can not control the robot in a proper way. 

However, I already checked several times the optimization algorithm(QP) and do not find the errors. But may be due to the fact of PID gains of angular velcocity in x axis(roll).

### Check the PID gains:

1. As we can see in the vx plot(the first picture): the PID gains are good enough so we just focus on the PID gains in roll
2. PID gains in angular velocity in x axis(roll)
![Image text](https://github.com/x1aoo/freefloating/raw/master/image/pure_roll.png)

As we can see in the last picture, before around 25s the roll could have a stable value but after that it has the big errors uncontrollable. I do not think it is due to the PID gains because we do not have any prove that the PID gains could lead to the fact that before 25s it has the stable value and after that it has big errors.

But what is the symbol of PID gains are correct? The stable begining(without desired value) or the stable tracking? I should check the desired performance.

### Turning PID gains:

1. The method
    1. First, let Ki = Kd =0, and turning Kp from 0 to larger until the systems begin to oscillate. And then make the Kp become less until the oscillate disappear. The suitable Kp is this Kp value * 0.6
    2. Set a bigger Ki and make the value become less until the systems begin to oscillate. Then make the Ki become bigger until the oscillate disappear. The suitable Ki value should be this Ki value * 1.5
2. Result
    + After turning PID gains, it shows that the oscillate never stop at the turning Kp and Ki process.

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/pure_angles.png)

However, we can see in the angles plot: after 40s it change rapidly. It satisfies the desired roll motion but we could compare to the non angular motion to test the desired angles change.

### Check angles change:

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/no_angles.png)
![Image text](https://github.com/x1aoo/freefloating/raw/master/image/no_roll.png)

It is very unusual that after around 40s, the angular has the big errors and I do not know where cause the errors. Since after 5s the robot arrived its desired depth and no change on other direction. So why the robot will go into this situation? And also, we could not confirm that the optimization process is correct due to the angles change is similar to the previous one. That is: after 40s the angles will change rapidly. So I will check if somewhere wrong in implementation leading this fact especially in the desired velocity setpoint or we need to change the work mode: it means that the control axis are not freely.

### Check the desired velocity setpoint especially in angular velocity:

There is no problem here.

### Change the work mode to velocity: change the setpoint of depth

- test the velocity control of z using depth(vz = 0.1 * (current_depth - desire_depth)

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/depth_velocity.png)

- the roll could not achieve whether we set the roll a value of the function 0.1 * (current_angle - desire_angle)

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/roll_velocity.png)

1. I do not know what cause the roll motion set point useless
2. I do not know what happens in 45s and 120s
3. The angles go correctly

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/angles_velocity.png)

### For now, I have no idea what I should check next step.

