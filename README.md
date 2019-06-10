# freefloating

The readme was created on 16th May, before that I already finished the main implementation part but there are some unknown errors here. And that is also the reason why I created this file, to debug...

The master thesis: Control of an Autunomous Underwater with Vectored Thrusters

/freefloating_gazebo : is to do the PID control and control allocation in gazebo
/freefloating_gazebo_demo : is the vectored thrusters model and the environment
/slider_publisher : is to publish the desired velocity/position or effort by slider

The plots are not in this folder and tha main problem now is the roll motion are not following the desired one.

## Debug report:

### Try to find the caurse of errors: 16th May

Linear velocity in x axis(vx) :
with desired angular velocity in x axis is always 0 and with linear velocity in x axis:

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/vx.png)
    **Fig. 1** Linear velocity in x axis

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/roll.png)
    **Fig. 2** Angular velocity in x axis
    
At the beginning, the actual linear velocity gose to the desired one and the angular velocity is equal to 0 but at the time:50 the angular velocity has the big errors and at the time 55 the linear velocity has the errors

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/angles.png)
    **Fig. 3** The thrusters' angles change
    
Since we do not set the desired angular velocity in x axis, the angles are not expected to have the big change like that but it may because the robot want to stablize the angular velocity(roll) but from the roll plot it do not have the position effect. It seems that the algorithm can not control the robot in a proper way.

However, I already checked several times the optimization algorithm(QP) and do not find the errors. But may be due to the fact of PID gains of angular velcocity in x axis(roll).

### Check the PID gains: 16th May

1. As we can see in the vx plot(the first picture): the PID gains are good enough so we just focus on the PID gains in roll
2. PID gains in angular velocity in x axis(roll)
![Image text](https://github.com/x1aoo/freefloating/raw/master/image/pure_roll.png)
    **Fig. 4** Turning PID
    
As we can see in the last picture, before around 25s the roll could have a stable value but after that it has the big errors uncontrollable. I do not think it is due to the PID gains because we do not have any prove that the PID gains could lead to the fact that before 25s it has the stable value and after that it has big errors.

But what is the symbol of PID gains are correct? The stable begining(without desired value) or the stable tracking? I should check the desired performance.

### Turning PID gains: 17th May

+ [] Set all the gains in roll into 0 and check the plot of vx and roll, it means that we do not control the roll motion check the vx still has some noise or not?

1. The method
    1. First, let Ki = Kd =0, and turning Kp from 0 to larger until the systems begin to oscillate. And then make the Kp become less until the oscillate disappear. The suitable Kp is this Kp value * 0.6
    2. Set a bigger Ki and make the value become less until the systems begin to oscillate. Then make the Ki become bigger until the oscillate disappear. The suitable Ki value should be this Ki value * 1.5
2. Result
    + After turning PID gains, it shows that the oscillate never stop at the turning Kp and Ki process.

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/pure_angles.png)

However, we can see in the angles plot: after 40s it change rapidly. It satisfies the desired roll motion but we could compare to the non angular motion to test the desired angles change.

### Check angles change: 16th May

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/no_angles.png)
![Image text](https://github.com/x1aoo/freefloating/raw/master/image/no_roll.png)

It is very unusual that after around 40s, the angular has the big errors and I do not know where cause the errors. Since after 5s the robot arrived its desired depth and no change on other direction. So why the robot will go into this situation? And also, we could not confirm that the optimization process is correct due to the angles change is similar to the previous one. That is: after 40s the angles will change rapidly. So I will check if somewhere wrong in implementation leading this fact especially in the desired velocity setpoint or we need to change the work mode: it means that the control axis are not freely.

### Check the desired velocity setpoint especially in angular velocity: 16th May

There is no problem here.

### Change the work mode to velocity: change the setpoint of depth: 16th May

- test the velocity control of z using depth(vz = 0.1 * (current_depth - desire_depth)

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/depth_velocity.png)

- the roll could not achieve whether we set the roll a value of the function 0.1 * (current_angle - desire_angle)

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/roll_velocity.png)

1. I do not know what cause the roll motion set point useless
    + check the optimization part.
2. I do not know what happens in 45s and 120s
3. The angles go correctly

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/angles_velocity.png)

### Check the Optimization part: the cost and function 17th May

![Image text](https://github.com/x1aoo/freefloating/raw/master/image/cost_rsum.png)
![Image text](https://github.com/x1aoo/freefloating/raw/master/image/cost_rsum_rate.png)

As can be seen, the cost in roll almost take up the whole part in total cost, it is means that the cost calculation has somewhere wrong or the algorithm would not reduce the cost in roll.

So I will first check： 
1. the cost calculation and if there is not anything wrong
    + There is no error here.
    
2. the optimization algorithm may have somewhere wrong.
    + the cost of roll reduce may cause the cost increase of other?
    + check the delta angles(done)
    + Find the errors in joint state publisher and the topic remap(done)
        + Now is try to publish the desired angles in to a suitable place.(done)
        + After test, the systems could publish the angles correctly although we could not see clearly in rqt graph
        + The result still wrong and I do not know where gose wrong.        
3. The potential method
    + [] Use velocity control in joint control instead of position control
        + If it works, to think about with velocity make it but position control
        + The f should not be 0 from the 3rd derivative function(Could we do not give the linear velocity in x axis)
### Next Step is to do the 3rd order derivative May 19th

## To do the 3rd order derivative method:
### Planning May 20th
+ Time:
    + Within 3 days: From 20th to 22nd May
        + [x] Plan(1) 20st
        + [x] Plan(2) 21st
        + [x] Plan(3) 21st
        + [] Plan(4) 21st - 22th
+ Plan:
    + [x] Check the function again carefully and clearly.
    + [x] Copy another code file to do the implementation
    + [x] Test with the desired trajectory tracking.
    + [] Find a better trajectory and fix the bugs
    
### 3rd order function May 20th
+ First use the simple trajectory: P = 0, w = constant = 10 degree/s in x;
+ Still have some bugs need to debug
    + [x] The initialize of Eigen Matrix has somewhere wrong.  May 21th
    + [x] Something wrong with the publisher of body efforts or the calculation results
    + [x] Something wrong with the initialization of desired position and initializtion of matrix
    + [] Move but still could not track the desired trajectory - may wrong in K or in desired trajectory
    + [] Or the Initialization problem
    + [x] joint velocity no effect and turn to the position setpoint method
    + [] body command no effect 
        + [] Why joint_command is published to /gazebo but thruster_command is to pid_control?
        
    + [x] Try to use the dynamic reconfigure to turn the parameter(Kp)
        + [x] After changing the name, I met bugs of "has no member named ‘kpdd’"
        + [x] Finish the case that with fix variable name and ranges
        + [x] Try to change the name of variable and ranges
        
    + [x] Check the desired angular variable state
        + [x] What the means of the each items of wd, rd and etc
            + [x] The terms of angular are respected to the matrix square
            + [x] The terms of eR is the rotation vector of rotation matrix
            + [x] The eR is different from the paper one
            + [x] The wd w are need to check