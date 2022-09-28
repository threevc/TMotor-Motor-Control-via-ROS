#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float32,Float64MultiArray,Int32
import time

class Trajectory:

    threshold: float

    def __init__(self, sub_topic_suffix = 1, pub_topic_suffix = 1, sub_topic = "motor_target_topic", pub_topic = "motor_control_topic"):

        self.sub_topic = sub_topic + str(sub_topic_suffix)
        self.sub = rospy.Subscriber(self.sub_topic, Float32, self.callback_motor)
        # self.sub1 = rospy.Subscriber("dynamixel1_cont", Float32, self.cb_mot)
        # self.sub2 = rospy.Subscriber("dynamixel2_cont", Float32, self.cb_mot2)

        self.pub_topic = pub_topic + str(pub_topic_suffix)
        self.pub = rospy.Publisher(self.pub_topic, Float64MultiArray, queue_size=10)
        # self.pub1 = rospy.Publisher('dynamixel1_control', Float64MultiArray, queue_size=10)
        # self.pub2 = rospy.Publisher('dynamixel2_control', Float64MultiArray, queue_size=10)

        self.mot_target = 0
        self.mot_current = 0
        self.mot_step = 1

        # self.mot_target = 0
        # self.mot_current = 0
        # self.mot_step = 1

        # self.mot2_target = 0
        # self.mot2_current = 0
        # self.mot2_step = 1

        self.threshold = 2

        self.prev_time = time.time()
        self.time = 1
        self.k = 1

    #with k 0.5 max speed is achieved

    def callback_motor(self,data):
        self.mot_target = data.data

    # def cb_mot2(self,data):
    #     self.mot2_target = data.data

    def publish_trajectory(self):
        if (self.mot_target - self.mot_current) > self.threshold:
        self.mot_current = self.mot_current + self.mot_step

        elif (self.mot_target - self.mot_current) < -self.threshold:
        self.mot_current = self.mot_current - self.mot_step

        # if (self.mot2_target - self.mot2_current) > self.threshold:
        # self.mot2_current = self.mot2_current + (self.mot2_step)

        # elif (self.mot2_target - self.mot2_current) < -self.threshold:
        # self.mot2_current = self.mot2_current - (self.mot2_step)

        pubs = Float64MultiArray()
        if self.threshold >= (self.mot_target - self.mot_current) >= -self.threshold:
            pubs.data = np.array([self.mot_current,0])
        else:
            pubs.data = np.array([self.mot_current,self.mot_step/self.time])

        # pubs2 = Float64MultiArray()
        # if (self.mot2_target - self.mot2_current) <= self.threshold and (self.mot2_target - self.mot2_current) >= -self.threshold:
        # pubs2.data = np.array([self.mot2_current,0])
        # else:
        # pubs2.data = np.array([self.mot2_current,self.mot2_step/self.t])

        self.pub.publish(pubs)
        # self.pub2.publish(pubs2)
        #print(self.mot_step/self.t)

    def set_time(self): 
        # self.prev_time = self.time
        self.time = time.time() - self.prev_time
        self.prev_time = self.time + self.prev_time


def main(argv):
    rospy.init_node("motor_trajectory_node")
    traj1 = Trajectory(sub_topic_suffix=1,pub_topic_suffix=1)
    traj2 = Trajectory(sub_topic_suffix=2,pub_topic_suffix=2)

    rate = rospy.Rate(5)

    # t_prev = time.time()

    while not rospy.is_shutdown():
        for traj in [traj1,traj2]:
            t = time.time()
            traj.set_time()
            traj.publish_trajectory()
        rate.sleep()
        # t_prev = t

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass