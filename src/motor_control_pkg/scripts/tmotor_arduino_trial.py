#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, String, UInt16
import time
import numpy as np
from matplotlib import pyplot as plt
import math
import random

# def torque_control(publisher_topic):
#     pub =rospy.Publisher('torque_topic',Float32,queue_size = 10)
#     # rospy.init_node('Toplevel')
#     # rate = rospy.Rate(100)## 1kHz
#     torque = 0.5
#     while not rospy.is_shutdown():
#         torque = torque+ random.randint(-1,1)*0.05
#         rospy.loginfo(torque)
#         pub.publish(torque)
#         rate.sleep()
pos_arr = []
time_arr = []
tor_arr = []
time1 = 0
err_arr = []


class TMotorControl:

    def __init__(self, motor_id=0x1):
        self.motor_id = motor_id
        # self.BAUDRATE = 57600
        # self.DEVICENAME = '/dev/ttyUSB0'

        self.kp = 1.5
        self.kd = -0.1
        self.ki = 2

        self.position = 0
        self.velocity = 0
        self.torque = 0

        self.position_error = 0
        self.velocity_error = 0
        self.torque_error = 0

        self.position_desired = 0
        self.velocity_desired = 0

        self.integral_error = 0

        self.integ = 0
        self.deriv = 0
        self.prop = 0

        self.switch = 0
        self.start_time = time.time()
        self.torque_read = 0

        self.pub = rospy.Publisher('torque_topic', Float32, queue_size=1)
        self.listener()
        rospy.Subscriber('input_switch', Float32, self.callback_switch)

    def callback_switch(self, data):
        self.position_desired = data.data

    def generate_pos_vel2(self):
        if self.switch == 0:
            pos = 2.0
            vel = 0.0
        else :
            pos = 5.0
            vel = 0.0
        return pos, vel

    def current_time(self):
        return time.time()-self.start_time

    def pid_control(self):
        self.position_desired, self.velocity_desired = self.generate_pos_vel2()

        self.position_error = self.position_desired - self.position
        self.velocity_error = self.velocity_desired - self.velocity

        if ((self.velocity < 0.005) and (self.velocity > -0.005)) and (
                (self.position_error < 0.02) and (self.position_error > -0.02)):
            self.ki = 1000
            self.integral_error = self.integral_error + self.position_error

        elif (self.velocity < 0.005) and (self.velocity > -0.005):
            self.ki = 10
            self.integral_error = self.integral_error + self.position_error
        else:
            self.ki = 0
            self.integral_error = 0
        # elif ((self.position_error < 0.005) and (self.position_error > -0.005)):
        #     self.ki = 0
        #     self.integral_error = 0 #self.integral_error + self.position_error
        if self.current_time() < 15:
            self.ki = 0
            self.integral_error = 0
        # self.ki  = 0
        # self.integral_error = self.integral_error + self.position_error

        self.prop = self.kp * self.position_error
        self.deriv = self.kd * self.velocity_error
        self.integ = self.ki * self.integral_error

        self.torque = self.prop + self.deriv + self.integ

        print('Err | Pos | Vel | P_Des | V_Des | Tor = ', round(self.position_error, 3), round(self.position, 2),
              round(self.velocity, 2), self.position_desired, self.velocity_desired, self.torque)

        self.pub.publish(self.torque)

    def pd_control(self):
        self.position_desired, self.velocity_desired = self.generate_pos_vel()

        self.position_error = self.position_desired - self.position
        self.velocity_error = self.velocity_desired - self.velocity

        self.prop = self.kp * self.position_error
        self.deriv = self.kd * self.velocity_error

        self.torque = self.prop + self.deriv
        # if (self.velocity < 0.5):
        #     self.torque = self.torque+5
        print('Err | Pos | Vel | P_Des | V_Des | Tor = ', round(self.position_error, 3), round(self.position, 2),
              round(self.velocity, 2), self.position_desired, self.velocity_desired, self.torque)
        # self.torque = self.torque + random.randint(-1, 1) * 0.05
        # rospy.loginfo("The torque supplied is %f", self.torque)
        # self.pub.publish(self.position_desired)
        self.pub.publish(self.torque)

    # def writeToMotor(self, torque):
    #     pub.publish(self.torque)

    # def map(self, x, in_min, in_max, out_min, out_max):
    #     x_scaled =int ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    #     x_mapped = x_scaled
    #     x_mapped = int(out_max) if x_scaled > out_max
    #     x_mapped = int(out_min) if x_scaled < out_min
    #     return x_mapped

    def generate_pos_vel(self):
        seconds = int(time.time())
        seconds = seconds - 100 * int(seconds / 100)
        # pos = float(math.sin(seconds*math.pi/6))
        pos = int(seconds / 4)
        while pos > 25:
            pos = pos - 25
        pos = pos - 12.5
        vel = float(0)
        return pos, vel

    def shutdown(self):
        print('shutdown')
        self.pub.publish(0)

    def callback_position(self, pos_in):
        self.position = pos_in.data
        # print('The position value received is ',pos_in.data)
        # print('The position value logged is                        ',self.position)
        # print('Received | Logged = ', pos_in.data , '  ',self.position)
        # rospy.loginfo("The position is %f", self.position)

    def callback_velocity(self, vel_in):
        self.velocity = vel_in.data

        # rospy.loginfo("The velocity is %f", self.velocity)

    def listener(self):
        rospy.Subscriber('position_feedback', Float32, self.callback_position)
        rospy.Subscriber('velocity_feedback', Float32, self.callback_velocity)
        # rospy.spinOnce()


if __name__ == '__main__':
    rospy.init_node('Toplevel')
    rate = rospy.Rate(100)  # 1000 Hz
    # pub = rospy.Publisher('torque_topic', Float32, queue_size=10)
    motor1 = TMotorControl()

    try:
        while not rospy.is_shutdown():
            motor1.pid_control()
            # pub.publish(motor1.torque)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    motor1.shutdown()