#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
           
            # Joint angle symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
      
            # Modified DH params
            DH = { alpha0:     0, a0:      0, d1:  0.75,
                   alpha1: -pi/2, a1:   0.35, d2:     0, q2:q2-pi/2,
                   alpha2:     0, a2:   1.25, d3:     0,
                   alpha3: -pi/2, a3: -0.054, d4:  1.50,
                   alpha4:  pi/2, a4:      0, d5:     0,
                   alpha5: -pi/2, a5:      0, d6:     0,
                   alpha6:     0, a6:      0, d7: 0.303, q7: 0}
            
            # Define Modified DH Transformation matrix
            def transformation_matrix( alpha, a, d, q):
                T = Matrix([[            cos(q),           -sin(q),           0,             a],
                            [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                            [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                            [                 0,                 0,           0,             1]])
                T = T.subs(DH)
                return(T)


            # Create individual transformation matrices
            T0_1 = transformation_matrix(alpha0, a0, d1, q1)
            T1_2 = transformation_matrix(alpha1, a1, d2, q2)
            T2_3 = transformation_matrix(alpha2, a2, d3, q3)
            T3_4 = transformation_matrix(alpha3, a3, d4, q4)
            T4_5 = transformation_matrix(alpha4, a4, d5, q5)
            T5_6 = transformation_matrix(alpha5, a5, d6, q6)
            T6_G = transformation_matrix(alpha6, a6, d7, q7)
            

            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            r1, r2, r3 = symbols('r1:4')
            R_x = Matrix([[        1,        0,        0, 0],
                          [        0,  cos(r1), -sin(r1), 0],
                          [        0,  sin(r1),  cos(r1), 0],
                          [        0,        0,        0, 1]])

            R_y = Matrix([[  cos(r2),        0,  sin(r2), 0],
                          [        0,        1,        0, 0],
                          [ -sin(r2),        0,  cos(r2), 0],
                          [        0,        0,        0, 0]])

            R_z = Matrix([[  cos(r3), -sin(r3),        0, 0],
                          [  sin(r3),  cos(r3),        0, 0],
                          [        0,        0,        1, 0],
                          [        0,        0,        0, 1]])
            
            R_corr = simplify(R_z*R_y)
            H = { r1:0, r2:-pi/2, r3:pi}
            R_corr = R_corr.subs(H)
           
            P_xyz = Matrix([[px],[py],[pz],[1]])
            
            R0_6 = simplify(R_z * R_y * R_x *R_corr.T)
            rpy = { r1:roll, r2:pitch, r3:yaw}
            R0_6 = R0_6.subs(rpy)
            D7 = Matrix([[0],[0],[d7],[1]])
            D7 = D7.subs(DH)
            WC_xyz = P_xyz - simplify(R0_6 * D7)            
		
#            print(WC_xyz)


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            
            theta1 = atan2(py,px)
            L1 = sqrt(WC_xyz[0,0]*WC_xyz[0,0]+WC_xyz[1,0]*WC_xyz[1,0])-a1
            L1 = L1.subs(DH)            
            L2 = WC_xyz[2,0]-d1
            L2 = L2.subs(DH)
            C = (L1*L1 + L2*L2 + a2*a2 - d4*d4 )/(2*a2)
            C = C.subs(DH)
            D = (L1*L1 + L2*L2 + d4*d4 - a2*a2 )/(2*d4)
            D = D.subs(DH)
            theta2 = atan2(L1,L2)-atan2(sqrt(L1*L1+L2*L2-C*C),C)
            theta2 = theta2*180/np.pi
#            print(theta2)
            if (theta2 > 85):
                 theta2 = atan2(L1,L2)+atan2(sqrt(L1*L1+L2*L2-C*C),C)
                 theta3 = atan2(-L2,L1)-atan2(sqrt(L1*L1+L2*L2-D*D),D)-theta2

            else:
                if (theta2 < -45):
                    theta2 = atan2(L1,L2)+atan2(sqrt(L1*L1+L2*L2-C*C),C)
                    theta3 = atan2(-L2,L1)-atan2(sqrt(L1*L1+L2*L2-D*D),D)-theta2

                else:
                    theta2 = atan2(L1,L2)-atan2(sqrt(L1*L1+L2*L2-C*C),C)
                    theta3 = atan2(-L2,L1)+atan2(sqrt(L1*L1+L2*L2-D*D),D)-theta2

#            theta2 = atan2(L1,L2)-atan2(sqrt(L1*L1+L2*L2-C),C)
#            print(theta2)
#            print(theta3)
            
            R0_3 = simplify(T0_1 * T1_2 * T2_3)
            R0_3 = R0_3.evalf(subs = {q1:theta1,q2:theta2,q3:theta3})
            R3_6 = simplify(R0_3.T * R0_6)
#            R3_6 = R3_6.evalf(subs = {q1:theta1,q2:theta2,q3:theta3})
            T3_6 = simplify(T3_4 * T4_5 * T5_6)
#            print(T3_6)
#            theta4 = atan2( R3_6[2,2],-R3_6[0,2])
#            theta5 = atan2(sqrt(R3_6[0,2]**2+R3_6[2,2]**2),R3_6[1,2])
#            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            theta4 = atan2(-R3_6[2,2], R3_6[0,2])
            theta5 = atan2(-1*sqrt(R3_6[0,2]**2+R3_6[2,2]**2),R3_6[1,2])
            theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            print(theta4)
            print(theta5)
            print(theta6)

	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
