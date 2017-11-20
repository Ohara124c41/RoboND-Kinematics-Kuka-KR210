# Project: Kuka-KR210 Kinematics Pick & Place Arm
___

##### Scholar: Christopher Ohara
##### Program: Udacity Robotics Nanodegree
___




### **Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/Screenshot%20from%202017-09-24%2015-35-04.png?raw=true
[image2]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/001.jpg?raw=true
[image3]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/002.jpg?raw=true
[image4]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/003.jpg?raw=true
[image5]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/Screenshot%20from%202017-09-24%2015-27-42.png?raw=true
[image6]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/Screenshot%20from%202017-09-24%2015-31-08.png?raw=true
[image7]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/Screenshot%20from%202017-09-24%2015-32-11.png?raw=true
[image8]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/Screenshot%20from%202017-09-24%2015-32-57.png?raw=true
[image9]: https://github.com/Ohara124c41/RoboND-Kinematics-Kuka-KR210/blob/master/images/Screenshot%20from%202017-09-24%2015-36-24.png?raw=true



---
### Writeup / README
___

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
___
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

First, we should become familiar with operating the robot in the simulator. A major goal of the project is to become familiar with operation using the forward kinematics, and then using the knowledge to implement inverse kinematics.

![alt text][image1]

___
#### 2. Derive the DH Parameter Table and create individual transformation matrices about each joint as well as a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The images below show the given configuration of the joints. The table contains the `four parameters` (alpha, a, d, theta) with respect to the relative links needed to derive the angles of operation. These parameters are inputted into the `generalized transformation matrix` to acquire the `individual transformation matrices.` The summation of the individual transformation matrices is the dot product of each individual transformation matrix. Note: The `end-effector` value (EE and G from the lecture videos) is represented as the "7" in the images since this is how I derived the equations. Also, theta is represented as "q" in the DH Table, since it is easier to program.

![alt text][image2]
![alt text][image3]


##### The given table, left in the report for readability as well as a way for readers to identify "alpha" as compared to "a":
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0

##### Next, convert to rotational matrices:
A snippet from the IK_server file is included:

```python
# Mathematical equivalents
T0_2 = T0_1 * T1_2
T0_3 = T0_2 * T2_3
T0_4 = T0_3 * T3_4
T0_5 = T0_4 * T4_5
T0_6 = T0_5 * T5_6

# Project transformation matrices to rotational matrices
R0_1 = T0_1[0:3,0:3]
R0_2 = T0_2[0:3,0:3]
R0_3 = T0_3[0:3,0:3]
R0_4 = T0_4[0:3,0:3]
R0_5 = T0_5[0:3,0:3]
R0_6 = T0_6[0:3,0:3]
R0_7 = T0_7[0:3,0:3]
```
___
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The image below shows the matrix used to incorporate the effects of `roll, pitch and yaw` (r,p,y). It also contains the correlation matrix that is dotted with the R_rpy matrix needed to define the `wrist center.` From here, the angles (theta) can be calculated with respect to the angle phi. These derivations are imported from the IK_server file.

![alt text][image4]

```python
    # Acquire angle theta1
    theta1 = atan2(wc[1],wc[0])

    # Use SSS triangle and Law of Sines with line segment lengths
    seg1_3 = a2_3
    seg1_6 = sqrt(pow((sqrt(wc[0] * wc[0]+wc[1] * wc[1]) - 0.35),2)+pow((wc[2] - 0.75), 2))
    seg3_6 = sqrt(d3_4**2+a3_4**2)

    # Take the inverse cosine to get the angle (phi)
    phi1 = acos((seg1_6 * seg1_6 + seg1_3 * seg1_3 -seg3_6 * seg3_6) / (2 * seg1_6 * seg1_3))
    phi2 = acos((seg3_6 * seg3_6 + seg1_3 * seg1_3 -seg1_6 * seg1_6) / (2 * seg3_6 * seg1_3))
    phi3 = acos((seg1_6 * seg1_6 + seg3_6 * seg3_6 -seg1_3 * seg1_3) / (2 * seg1_6 * seg3_6))

    # Calculate theta2 and theta3
    theta2 = pi/2 - phi1 - atan2((wc[2]-0.75),(sqrt(wc[0] * wc[0]+wc[1] * wc[1]) - 0.35))
    theta3 = pi/2 - phi2 - 0.036

    # Calculate rotational matrices
    R_rpy0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R_rpy3_6 = R_rpy0_3.T * R_rpy0_6

    # Use the previous information to acquire the remaining angles (theta)
    theta4 = atan2(R_rpy3_6[2,2], -R_rpy3_6[0,2])
    theta5 = atan2(sqrt(R_rpy3_6[0,2]*R_rpy3_6[0,2] + R_rpy3_6[2,2]*R_rpy3_6[2,2]),R_rpy3_6[1,2])
    theta6 = atan2(-R_rpy3_6[1,1],R_rpy3_6[1,0])
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. To receive a passing submission for the project, the code must guide the robot to successfully complete 8/10 pick and place cycles.

The final result was **9/10** successful pick and place cycles, as shown in the images below. One thing to note is, there was an error when running the project in autonomous mode (such that the robotic arm automatically completed each step of the cycle). The Slack channel reported other students that had this issue as well (probably related to the computational speed of the processor) which could be resolved by implementing an offset (delay) since the real-time event is time invariant. However, this was overcome by manual operation, since when I tried to implement the delay, I still had poor results (the arm would either not grab the cylinder, nudge it and move on to the next cycle or drop it on the way to the bin).

Overall, implementation is straight forward, as the user simply needs to input the mathematical formulae coherently.   

![alt text][image5]
![alt text][image6]
![alt text][image7]
![alt text][image8]

Finally, an image of the "empty joint" error is added, for visual representation:
![alt text][image9]
