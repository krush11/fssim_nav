#! /usr/bin/env python3
import math

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from fssim_nav.msg import PIDInput

pub = rospy.Publisher('nav', AckermannDriveStamped, queue_size=1)

kp = 10.
kd = 0.01
kp_vel = 42.0
kd_vel = 0.0

ki = 0.0
servo_offset = 18.0*math.pi/180
prev_error = 0.0
error = 0.0
integral = 0.0
vel_input = 1.0
avg_vel = 0.0
iteration = 0.0
total = 0.0


def control(data):
    global integral
    global prev_error
    global vel_input
    global kp
    global ki
    global kd
    global kd_vel
    global kp_vel
    global avg_vel
    global iteration
    global total
    velocity = data.pid_vel
    angle = servo_offset
    error = 5*data.pid_error
    print("Error Control", error)
    if error != 0.0:
        # if abs(error - prev_error)>0.5:
        # 	integral = integral + error
        control_error = kp*error + kd*(error - prev_error)  # + ki*integral
        print("Control", control_error)
        # integral = integral/1.3
        # print "Control error",control_error
        angle = angle + control_error*np.pi/180
        # print "Control error",control_error
        control_error_vel = kp_vel*error + kd_vel*(error - prev_error)
        # print "Control error velocity",control_error_vel
        # velocity = velocity - abs(control_error_vel)/10
        velocity = velocity + abs(control_error_vel)

    prev_error = error

    if angle > 30*np.pi/180:
        angle = 30*np.pi/180
    if angle < -30*np.pi/180:
        angle = -30*np.pi/180

    if angle > 20*np.pi/180 or angle < -20*np.pi/180:
        velocity = 1.0
    if angle >= 10*np.pi/180 or angle <= -10*np.pi/180:
        velocity = 1.5
    if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
        velocity = 2
    if velocity < 0:
        velocity = 1
    if velocity > 2.5:
        velocity = 2

    velocity *= 3.5
    angle /= 1.25

    iteration+=1
    total += velocity
    avg_vel = total/iteration

    print("Velocity", velocity)
    print("Average Velocity = ", avg_vel)
    print("Angle", angle)
    # print("total = ", total)
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'cmd_vel'
    msg.drive.speed = velocity
    msg.drive.steering_angle = angle
    pub.publish(msg)


def listener():
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", PIDInput, control)
    rospy.spin()


if __name__ == '__main__':
    print("Listening to error for PID")
    listener()
