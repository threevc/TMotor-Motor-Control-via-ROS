#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String, UInt16
import time
import math
import random

previous_time = 0


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

class TMotorControl:

    def __init__(self, motor_id=0x1):
        self.motor_id = motor_id
        # self.BAUDRATE = 57600
        # self.DEVICENAME = '/dev/ttyUSB0'

        self.kp = 0.8
        self.kd = -0.1

        self.position = 0
        self.velocity = 0
        self.torque = 0

        self.position_error = 0
        self.velocity_error = 0
        self.torque_error = 0

        self.position_desired = 0
        self.velocity_desired = 0

        self.pub = rospy.Publisher('torque_topic', Float32, queue_size=1)

    def pd_control(self):
        self.position_desired, self.velocity_desired = self.generate_pos_vel()

        rospy.loginfo("The position desired  is %f", self.position_desired)
        rospy.loginfo("The velocity desired  is %f", self.velocity_desired)
        self.listener()
        self.position_error = self.position_desired - self.position
        self.velocity_error = self.velocity_desired - self.velocity

        prop = self.kp * self.position_error
        deriv = self.kd * self.velocity_error

        self.torque = prop + deriv
        # self.torque = self.torque + random.randint(-1, 1) * 0.05
        rospy.loginfo("The torque supplied is %f", self.torque)
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

    def callback_position(self, value):
        self.position = value.data
        print('The position value received is ',value.data)
        print('The position value logged is   ',self.position)
        #rospy.loginfo("The position is %f", self.position)

    def callback_velocity(self, data):
        self.velocity = value.data

        rospy.loginfo("The velocity is %f", self.velocity)

    def listener(self):
        rospy.Subscriber('position_feedback', Float32, self.callback_position)
        rospy.Subscriber('velocity_feedback', Float32, self.callback_velocity)
        # rospy.spinOnce()


if __name__ == '__main__':
    rospy.init_node('Toplevel')
    rate = rospy.Rate(1000)  # 1000 Hz
    # pub = rospy.Publisher('torque_topic', Float32, queue_size=10)
    motor1 = TMotorControl()

    try:
        while not rospy.is_shutdown():
            motor1.pd_control()
            # pub.publish(motor1.torque)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    motor1.shutdown()
