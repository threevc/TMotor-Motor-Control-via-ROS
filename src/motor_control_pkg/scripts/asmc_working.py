#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float32,Float64MultiArray,Int32
import time

# Control Equation - u = -(lambda)*s + rho*signum(s)
# Lambda

class asmc:


    def __init__(self):
        self.Kp0 = np.array([1.0, 1.0, 1.0])
        self.Kp1 = np.array([2.0, 2.0, 1.0])
        self.Lam = np.array([2.2, 2.2, 5.0])  # Change 1
        self.Phi = np.array([1.5, 1.5, 1.1])
        self.M = 0.4                          # Change 2
        self.alpha_0 = np.array([1,1,1])
        self.alpha_1 = np.array([3,3,3])
        self.alpha_m = 0.05                   # Change 3
        self.v = 0.1
        self.prev_time = time.time()


    def th_des(self):
        dt = rospy.get_time() - self.prev_time
        self.prev_time = self.prev_time + dt
        if dt > 0.04:
            dt = 0.04

        curPos = self.vector2Arrays(self.cur_pose.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position)
        curVel = self.vector2Arrays(self.cur_vel.twist.linear)

        errPos = curPos - desPos
        errVel = curVel - self.desVel
        sv = errVel + np.multiply(self.Phi, errPos)
        #print(errPos)
        #print("------------------")

        if self.armed:
            self.Kp0 += (sv - np.multiply(self.alpha_0, self.Kp0))*dt
            self.Kp1 += (sv - np.multiply(self.alpha_1, self.Kp1))*dt
            self.Kp0 = np.maximum(self.Kp0, 0.0001*np.ones(3))
            self.Kp1 = np.maximum(self.Kp1, 0.0001*np.ones(3))
            self.M += (-sv[2] - self.alpha_m*self.M)*dt
            self.M = np.maximum(self.M, 0.1)
            # print(self.M)

        Rho = self.Kp0 + self.Kp1*errPos

        delTau = np.zeros(3)
        delTau[0] = Rho[0]*self.sigmoid(sv[0],self.v)
        delTau[1] = Rho[1]*self.sigmoid(sv[1],self.v)
        delTau[2] = Rho[2]*self.sigmoid(sv[2],self.v)

        des_th = -np.multiply(self.Lam, sv) - delTau + self.M*self.gravity

        # self.array2Vector3(sv, self.data_out.sp)
        # self.array2Vector3(self.Kp0, self.data_out.Kp_hat)
        # self.array2Vector3(errPos, self.data_out.position_error)
        # self.array2Vector3(errVel, self.data_out.velocity_error)
        # self.array2Vector3(delTau, self.data_out.delTau_p)
        # self.array2Vector3(Rho, self.data_out.rho_p)
        # self.data_out.M_hat = self.M


        if np.linalg.norm(des_th) > self.max_th:
            des_th = (self.max_th/np.linalg.norm(des_th))*des_th

        return des_th
