## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/joint3.png
[image2]: ./misc_images/z_pi.png
[image3]: ./misc_images/x_pi2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

##### Evaluating the kr210.urdf.xacro file  



jointname | parent_link | child link  | x | y | z | roll | pitch | yow |  min | max |
--- | --- | --- | --- |--- | --- | --- | --- | ---| --- | ---   
joint_1 | base_link| link1 | 0 | 0 | 0.33 | 0 | 0 | 0 | -185 | 185
joint_2 | link1 | link2 | 0.35 | 0 | 0.42 | 0 | 0 | 0 | -45 | 85
joint_3 | link2 | link3 | 0 | 0 | 1.25 | 0 | 0 | 0 | -210 | 155-98
joint_4 | link3 | link4 | 0.96 | 0 | -0.054 |  0 | 0 | 0 | -350 | 350
joint_5 | link4 | link5 | 0.54 | 0 | 0 | 0 | 0 | 0 | -125 | 125
joint_6 | link5 | link6 | 0.193 | 0 | 0 | 0 | 0 | 0 | -350 | 350
gripper_joint| link6 | gripper_link| 0.11| 0 | 0 | 0 |  0 | 0 | |

##### obtaining the DH parameters

αi-1 is the angle between zi-1 and zi.  
ai-1 is the distance zi-1 to zi measured along the x1_axis.  
di is the distance between xi-1 and xi.  
θi is the angle between xi-1 to xi.  

![alt text][image1]

##### the DHparameter table

i | $\alpha_{i-1}$| $a_{i-1}$ | $d_i$ | $\theta_i$
--- | --- | --- | ---|---
1 | 0     | 0      | 0.75 | q1
2 | -pi/2 | 0.35   | 0    | q2-pi/2
3 |  0    | 1.25   | 0    | q3
4 | -pi/2 | -0.054 | 1.50 | q4
5 |  pi/2 | 0      | 0    | q5
6 | -pi/2 | 0      | 0    | q6
7 | 0     | 0      | 0.303| 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

 ##### homogeneous transforms

 $_{i}^{i-1}T$ = $R_x(α_{i-1}) D_x(a_{i-1}) R_z(θ_{i}) D_z(d_{i})$

define the function of transforms

```python
  def transformation_matrix( alpha, a, d, q):
      T = Matrix([[            cos(q),           -sin(q),           0,             a],
                  [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                  [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                  [                 0,                 0,           0,             1]])
      T = T.subs(DH)
      return(T)
```
using ```transformation_matrix()```create individual transformation matrices about each joint.

#####generating a generalized homogeneous transform between base_link and gripper_link


 end-effector(gripper) orientation represent $\begin{equation}R_z(roll) * R_y(pitch)*R_x(yow) =   
 \begin{bmatrix}
  r_{11} &r_{12}&r_{13}\\
  r_{21} &r_{22}&r_{23}\\
  r_{31} &r_{32}&r_{33}\\
  \end{bmatrix}
  \end{equation}$

  generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


 $\begin{equation}
_{EE}^0 T =
 \begin{bmatrix}
 r_{11} &r_{12}&r_{13}&P_x\\
 r_{21} &r_{22}&r_{23}&P_y\\
 r_{31} &r_{32}&r_{33}&P_z\\
 0      &0     &0     &1
 \end{bmatrix}
 \end{equation}$


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


define the $R_{corr}$

$R_{corr} = R_z(\pi) * R_y(\pi/2)$

$_6^0R * R_{corr}=_6^0R * R_z(\pi) * R_y(\pi/2)$

$_6^0R * R_z(\pi)$ represent below

![alt text][image2]

$_6^0R * R_z(\pi)*R_y(\pi/2)$ represent below

![alt text][image3]

 $_6^0R *R_{corr}$ is end-effector(gripper) orientation.

 $_6^0R = R_z(roll) * R_y(pitch)*R_x(yow)*R_{corr}.T$

calculate the location the spherical wrist(WC)

$\begin{equation}
WC = \begin{bmatrix}
P_{x}\\
P_y \\
P_z\\
1
\end{bmatrix}-_6^0R *
\begin{bmatrix}
0\\
0\\
0\\
d7
\end{bmatrix}
\end{equation}$

$\theta_1 = atan2(Py,Px)$

$L1 = \sqrt{WC_x^ 2+WC_y^2}-a_1$  
$L2 = WC_z-d1$  
$C = (L1^2 + L2^2 + a_2^2 - d_4^2 )/2a_2$  
$D = (L1^2 + L2^2 + d_4^2 - a_2^2 )/2d_4$  

$\theta_2 = atan2(L1,L2)-atan2(\sqrt{L1^2+L2^2-C^2},C)$  
$\theta_3 = atan2(-L2,L1)+atan2(\sqrt{L1^2+L2^2-D^2},D)-\theta_2$

or  
$\theta_2 = atan2(L1,L2)+atan2(\sqrt{L1^2+L2^2-C^2},C)$  
$\theta_3 = atan2(-L2,L1)-atan2(\sqrt{L1^2+L2^2-D^2},D)-\theta_2$


$_3^0R = _1^0T * _2^1T * _3^2T$  

$_6^3R = _3^0R.T * _6^0R$
$_6^3T = _4^3T * _5^4T * _6^5 T$

$\theta_4 = atan2( _6^3R[2,2],-_6^3R[0,2])$  
$\theta_5 = atan2(\sqrt{_6^3R[0,2]^2+_6^3R[2,2]^2},_6^3R[1,2])$  
$\theta_6 = atan2(-_6^3R[1,1], _6^3R[1,0])$  

or  

$\theta_4 = atan2(- _6^3R[2,2],_6^3R[0,2])$  
$\theta_5 = atan2(-\sqrt{_6^3R[0,2]^2+_6^3R[2,2]^2},_6^3R[1,2])$  
$\theta_6 = atan2(_6^3R[1,1], -_6^3R[1,0])$



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

difine the default $\theta_2$ and $\theta_3$   
$\theta_2 = atan2(L1,L2)-atan2(\sqrt{L1^2+L2^2-C^2},C)$  
$\theta_3 = atan2(-L2,L1)+atan2(\sqrt{L1^2+L2^2-D^2},D)-\theta_2$



$\theta_4$~$\theta_6$difine  
$\theta_4 = atan2(- _6^3R[2,2],_6^3R[0,2])$  
$\theta_5 = atan2(-\sqrt{_6^3R[0,2]^2+_6^3R[2,2]^2},_6^3R[1,2])$  
$\theta_6 = atan2(_6^3R[1,1], -_6^3R[1,0])$

this code guide the robot to successfully complete 8/10 pick and place cycles.
