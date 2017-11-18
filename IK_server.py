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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ## D-H parameters syms - length, offset and rotational angles
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Joint Angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Distance and Angle (in meters and radians)
        d0_1 = 0.75
        d3_4 = 1.5
        d4_7 = 0.303

        a1_2 = 0.35
        a2_3 = 1.25
        a3_4 = -0.054

        alpha1_2 = -pi/2
        alpha3_4 = -pi/2
        alpha4_5 =  pi/2
        alpha5_6 = -pi/2

        # D-H parameters specifics for KUKA KR210 from FWD Kinematics Section
        DH_Table = {    alpha0:      0, a0:      0,   d1:  0.75,   q1:         q1,
                        alpha1:  -pi/2, a1:   0.35,   d2:     0,   q2: -pi/2 + q2,
                        alpha2:      0, a2:   1.25,   d3:     0,   q3:         q3,
                        alpha3:  -pi/2, a3: -0.054,   d4:   1.5,   q4:         q4,
                        alpha4:   pi/2, a4:      0,   d5:     0,   q5:         q5,
                        alpha5:  -pi/2, a5:      0,   d6:     0,   q6:         q6,
                        alpha6:      0, a6:      0,   d7: 0.303,   q7:          0,}

        # Define Modified DH Transformation matrix
        def TF_MAT (alpha, a, d, q):
            TF = Matrix([[             cos(q),           -sin(q),           0,             a],
                          [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                          [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                          [                 0,                 0,           0,             1]])
            return TF

        # Create individual transformation matrices
        T0_1 = TF_MAT(alpha0,a0,d1,q1).subs(DH_Table)
        T1_2 = TF_MAT(alpha1,a1,d2,q2).subs(DH_Table)
        T2_3 = TF_MAT(alpha2,a2,d3,q3).subs(DH_Table)
        T3_4 = TF_MAT(alpha3,a3,d4,q4).subs(DH_Table)
        T4_5 = TF_MAT(alpha4,a4,d5,q5).subs(DH_Table)
        T5_6 = TF_MAT(alpha5,a5,d6,q6).subs(DH_Table)
        T6_7 = TF_MAT(alpha6,a6,d7,q7).subs(DH_Table)

        # Final product of matrix multiplication
        T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7

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

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()

            # px,py,pz are the end-effector positions
            # r, p, y are the end-effector orientation (roll, pitch, yaw)
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # Quaternions are represented as (x,y,x,w) where w is the angle (omega)
            (r, p, y) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                req.poses[x].orientation.z, req.poses[x].orientation.w])

            # R_rpy = roll, pitch, yaw
            R_rpy =  Matrix([[cos(y)*cos(p), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r)],
                            [sin(y)*cos(p), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r)],
                            [-sin(p),         cos(p)*sin(r),                             cos(p)*cos(r)]])

            # Compute correlation matrix
            R_correlation = Matrix([[0,0,1],
                                   [0,-1,0],
                                   [1,0,0]])
            # Calculate the origin matrix by the rpy and correlation transpose
            R_rpy0_6 = R_rpy*(R_correlation.T)

            # Calculate the end-effector matrix
            EE = Matrix([[px],
                         [py],
                         [pz]])

            # Calculate wrist center
            wc = EE - d4_7 * R_rpy0_6[:,2]

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

            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
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
