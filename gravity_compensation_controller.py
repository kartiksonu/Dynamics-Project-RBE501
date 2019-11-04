#!/usr/bin/env python

import rospy
import std_msgs.msg
import yaml
import numpy as np


from ambf_msgs.msg import ObjectState, ObjectCmd
from dynamic_param_parser import Inverse_dynamics_calc_func
from dynamic_param_parser import Bodies_count
from geometry_msgs.msg import Vector3

#our global variables
state_msg = ObjectState()
#using active as a flag to use the gravity compensation controller
active = True
Q_vals = []


# Kp and Kd values
Kp = 100
Kd = 0.3

# ROS Subscriber callback function
def get_joint_values(data):
	global state_msg, Q_vals
	Q_vals = data.joint_positions
	#print "Q_values are ", Q_vals




def controller():
	global active, Kp, Kd
	sub = rospy.Subscriber('/ambf/env/base/State', ObjectState, get_joint_values, queue_size=1)
	pub = rospy.Publisher('/ambf/env/base/Command', ObjectCmd, queue_size=1)
	rospy.init_node('ambf_gravitycontrol_test')
	rate = rospy.Rate(1000)   #1000hz


	# calculating the time difference
	dt = 0.001
	cur_time = rospy.Time.now().to_sec()

	cmd_msg = ObjectCmd()
	cmd_msg.enable_position_controller = False
	cmd_msg.position_controller_mask = [False]

	while not rospy.is_shutdown():

		#Read time from ROS
		last_time = cur_time
		cur_time = rospy.Time.now().to_sec()
		dt = cur_time - last_time
		print "loop freq is " , 1/dt


		if dt < 0.001:
			dt = 0.001
		if active:
			# Reading current position value of all the links

			t1              = rospy.Time.now().to_sec()
			prev_pos        = np.asarray(Q_vals) 
			t2              = rospy.Time.now().to_sec()
			cur_pos         = np.asarray(Q_vals)
			time_btwn_poses = t2 -t1

			pose_diff = cur_pos - prev_pos
			velocity_pid = pose_diff*dt

			velocity_val = np.zeros(7)
			accel_val    = np.zeros(7)

			torque_gravity_comp = Inverse_dynamics_calc_func(np.zeros(7), velocity_val, accel_val)

			torque_pid = Kd*(velocity_pid-velocity_val) + Kp*pose_diff

			torque = 10*torque_gravity_comp + torque_pid
			# print(" torque is ", len(torque))

			#define header
			Header = std_msgs.msg.Header()
			Header.stamp = rospy.Time.now()
			cmd_msg.header = Header
			# cmd_msg.joint_cmds = [torque[0], torque[1], torque[2], torque[3], torque[4], torque[5], torque[6]]
			cmd_msg.joint_cmds = [ 0, torque[1], 0, 0, 0, 0, 0]

			print(" torque is ", cmd_msg.joint_cmds)

			pub.publish(cmd_msg)
			rate.sleep()

	rospy.spin()



if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass































