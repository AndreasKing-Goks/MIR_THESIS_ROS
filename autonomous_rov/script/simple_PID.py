#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken

import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
import time
import sys
import argparse

# ---------- Global Variables ---------------

set_mode = [0]*3
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

#Conditions
init_a0 = True
init_p0 = True
arming = False

# Orientation Parameter
angle_wrt_startup = [0]*3
angle_roll_a0 = 0.0
angle_pitch_a0 = 0.0
angle_yaw_a0 = 0.0

# Pressure parameters
depth = 0
depth_p0 = 0

enable_depth = False 
enable_ping = True 

Vmax_mot = 1900
Vmin_mot = 1100

# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 

# ------ Controller Setpoints -------

# Position setpoints
ref_x = 0.0				# surge position
ref_y = 0.0				# sway position
ref_z = -0.5			# heave position

# Orientation setpoints
ref_roll = 0.0			# roll orientation
ref_pitch = 0.0			# pitch orientation
ref_yaw = 0.0			# yaw orientation

# ---------- Joystick ---------------

def joyCallback(data):
	global arming
	global set_mode
	global init_a0
	global init_p0
	global Sum_Errors_Vel
	global Sum_Errors_angle_yaw
	global Sum_Errors_depth

	# Joystick buttons
	btn_arm = data.buttons[7]  # Start button
	btn_disarm = data.buttons[6]  # Back button
	btn_manual_mode = data.buttons[3]  # Y button
	btn_automatic_mode = data.buttons[2]  # X button
	btn_corrected_mode = data.buttons[0]  # A button


	# Disarming when Back button is pressed
	if (btn_disarm == 1 and arming == True):
		arming = False
		armDisarm(arming)

	# Arming when Start button is pressed
	if (btn_arm == 1 and arming == False):
		arming = True
		armDisarm(arming)

	# Switch manual, auto and correction mode
	if (btn_manual_mode and not set_mode[0]):
		set_mode[0] = True
		set_mode[1] = False
		set_mode[2] = False	
		rospy.loginfo("Manual Mode")

	if (btn_automatic_mode and not set_mode[1]):
		set_mode[0] = False
		set_mode[1] = True
		set_mode[2] = False		
		rospy.loginfo("Automatic Mode")

		# Run Algorithm
		rospy.Timer(rospy.Duration(0.1), lambda event:stabilize())

	if (btn_corrected_mode and not set_mode[2]):
		init_a0 = True
		init_p0 = True

		# set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
		set_mode[0] = False
		set_mode[1] = False
		set_mode[2] = True
		# rospy.loginfo("Correction Mode")

		# Run Algorithm
		rospy.loginfo("Mode has not been set up")


def armDisarm(armed):
	# This functions sends a long command service with 400 code to arm or disarm motors
	if (armed):
		# print('im armed')
		rospy.wait_for_service('/USV/mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('/USV/mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Arming Succeeded")
		except rospy.ServiceException as e:
			rospy.loginfo("Except arming")
	else:
		# print('im disarmed')
		rospy.wait_for_service('/USV/mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('/USV/mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Disarming Succeeded")
		except rospy.ServiceException as e:
			rospy.loginfo("Except disarming")	


# ---------- Functions --------------
class PID:
	def __init__(self, setpoint, Kp=0, Ki=0, Kd=0, sample_time=0.1):
		# Define setpoint
		self.setpoint = setpoint
	
		# Define PID gains
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd

		# Define time sample and initial time
		self.sample_time = sample_time
		self.init_time = time.time()
	
		# Define initial proportional error, and integral action
		self.proportional_error = 0
		self.integral_act = 0
	
	def compute(self, feedback_value):
		# Define current time
		current_time = time.time()
	
		# Compute elapsed time between initial time and consequent time
		elapsed_time = current_time - self.init_time
		# print(elapsed_time)

		# Compute proportional error
		error = self.setpoint - feedback_value

		# Compute error derivative
		der_error = error - self.proportional_error
		# print(der_error)
		# print(elapsed_time)
		derivative = (error / elapsed_time) if elapsed_time > 0 else 0
		# print(derivative)

		# Compute integral act
		self.integral_act += error * elapsed_time
		# print(self.integral_act)
	
		# Compute tau_cmd
		proportional_tau = self.Kp * error
		# print('proportional_tau = ', proportional_tau)
		integral_tau = self.Ki * self.integral_act
		# print('integral_tau = ', integral_tau)
		derivative_tau = self.Kd * derivative
		# print('derivative_tau = ', derivative_tau)

		tau_cmd = proportional_tau + integral_tau + derivative_tau
				
		# Reset time 
		self.init_time = current_time
		self.proportional_error = error

		return tau_cmd, error


def velCallback(cmd_vel):
	global set_mode

	# Only continue if Manual Mode is enabled
	if (set_mode[1] or set_mode[2]):
		# Set motors to idle when
		setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
		return

	# Extract cmd_vel message
	roll_left_right 	= mapValueScalSat(cmd_vel.angular.x)
	yaw_left_right 		= mapValueScalSat(-cmd_vel.angular.z)
	ascend_descend 		= mapValueScalSat(cmd_vel.linear.z)
	forward_reverse 	= mapValueScalSat(cmd_vel.linear.x)
	lateral_left_right 	= mapValueScalSat(-cmd_vel.linear.y)
	pitch_left_right 	= mapValueScalSat(cmd_vel.angular.y)

	# If no joystick input, set motors to neutral
	if all(v == 1500 for v in [roll_left_right, yaw_left_right, ascend_descend, forward_reverse, lateral_left_right, pitch_left_right]):
		setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
	else:
		# Log the cmd_vel message and mapped values
		setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)


def depthCallback(data):
	global depth

	# Only continue if Automatic Mode and Correction Mode are enabled
	if set_mode[0]:
		return

	# Get the depth
	depth = data.data

# def pressureDepthCallback(data):
# 	global depth
# 	global init_p0
# 	global depth_p0

# 	# Only continue if Automatic Mode and Correction Mode are enabled# print('im armed')

# 	depth = (pressure - 101300)/(rho*g) - depth_p0

def imuCallback(data):
	global angle_roll_a0
	global angle_pitch_a0
	global angle_yaw_a0
	global angle_wrt_startup
	global init_a0
	global p
	global q
	global r

	# Only continue if Automatic Mode and Correction Mode are enabled
	if set_mode[0]:
		return

	orientation = data.orientation
	angular_velocity = data.angular_velocity

	## Extraction of yaw angle
	q = [orientation.x, orientation.y, orientation.z, orientation.w]
	euler = tf.transformations.euler_from_quaternion(q)
	angle_roll = euler[0]
	angle_pitch = euler[1]
	angle_yaw = euler[2]

	if (init_a0):
		# at 1st execution, init
		angle_roll_a0 = angle_roll
		angle_pitch_a0 = angle_pitch
		angle_yaw_a0 = angle_yaw
		init_a0 = False

	angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	
	angle = Twist()
	angle.angular.x = angle_wrt_startup[0] * math.pi/180
	angle.angular.y = angle_wrt_startup[1] * math.pi/180
	angle.angular.z = angle_wrt_startup[2] * math.pi/180

	# Publish the orientation
	# pub_angle_degre.publish(angle)
	pub_angle_rad.publish(angle)

	## Extraction of angular velocity
	p = angular_velocity.x
	q = angular_velocity.y
	r = angular_velocity.z

	vel = Twist()
	vel.angular.x = p
	vel.angular.y = q
	vel.angular.z = r

	# Publish angular velocity
	pub_angular_velocity.publish(vel)

def stabilize():
	global angle_wrt_startup, depth
	global pub_msg_override, set_mode

	# Only continue if Automatic Mode and Correction Mode are enabled
	if set_mode[0]:
		return

	# Get the roll and pitch data
	roll = angle_wrt_startup[0] 	# degree
	pitch = angle_wrt_startup[1]	# degree

	# print(depth)



	# PID controllers for each degree of freedom
	
	## Stabile 1 (not so good, stock)
	# depth_PID = PID(setpoint=ref_z, Kp=250, Ki=150, Kd=0.001)
	# roll_PID = PID(setpoint=ref_roll, Kp=50, Ki=20, Kd=0.00005)
	# pitch_PID = PID(setpoint=ref_pitch, Kp=50, Ki=20, Kd=0.00005)

	## best then stabile 2
	# depth_PID = PID(setpoint=ref_z, Kp=280, Ki=180, Kd=0.0012)
	# roll_PID = PID(setpoint=ref_roll, Kp=50, Ki=20, Kd=0.00005)
	# pitch_PID = PID(setpoint=ref_pitch, Kp=50, Ki=20, Kd=0.00005)

	## Best but converge to -0.6
	# depth_PID = PID(setpoint=ref_z, Kp=280, Ki=200, Kd=0.0012)
	# roll_PID = PID(setpoint=ref_roll, Kp=50, Ki=20, Kd=0.0001)
	# pitch_PID = PID(setpoint=ref_pitch, Kp=50, Ki=30, Kd=0.0001)

	## Converge to -0.6, way better oscillation damping in pitch and heave
	depth_PID = PID(setpoint=ref_z, Kp=280, Ki=200, Kd=0.0012)
	roll_PID = PID(setpoint=ref_roll, Kp=50, Ki=20, Kd=0.0001)
	pitch_PID = PID(setpoint=ref_pitch, Kp=100, Ki=80, Kd=0.0011)

	# depth_PID = PID(setpoint=ref_z, Kp=280, Ki=300, Kd=0.0012)
	# roll_PID = PID(setpoint=ref_roll, Kp=100, Ki=50, Kd=0.0001)
	# pitch_PID = PID(setpoint=ref_pitch, Kp=100, Ki=100, Kd=0.0011)



	# Compute control body force
	[tau_depth, e_depth] = depth_PID.compute(depth)
	[tau_roll, e_roll] = roll_PID.compute(np.deg2rad(roll))
	[tau_pitch, e_pitch] = pitch_PID.compute(np.deg2rad(pitch))
	tau_stb = np.array([0, 0, tau_depth, tau_roll, tau_pitch, 0])

	# print('tau_depth', tau_depth)
	# print('tau_roll', tau_roll)
	# print('tau_pitch', tau_pitch)

	# Publish commanded thruster output (NEED TO ASK IF WE DON'T NEED THE THRUSTER ALLOCATION MATRIX)
	tau_stb_msg = Float64MultiArray()
	tau_stb_msg.data = tau_stb
	pub_tau_stb.publish(tau_stb_msg)

	# Update init signal
	init_signal = 1500	

	# Send PWM commands to motors --> Should be done with a proper mapping
	tau_depth_signal = int(init_signal + tau_depth)
	tau_roll_signal = int(init_signal + tau_roll)
	tau_pitch_signal = int(init_signal + tau_pitch)

	# print('depth = ', depth)
	# print('roll = ', roll)
	# print('pitch = ', pitch)
	# # print('init_signal = ', init_signal)
	print('e_depth = ', e_depth)
	print('tau_depth = ', tau_depth)
	# print('tau_depth_signal = ', tau_depth_signal)
	# print('tau_roll_signal = ', tau_roll_signal)
	# print('tau_pitch_signal = ', tau_pitch_signal)
	
	setOverrideRCIN(1500, 1500, tau_depth_signal, tau_roll_signal, tau_pitch_signal, 1500)

# -------- Utilities ------------

def mapValueScalSat(value):
	# Correction_Vel and joy between -1 et 1
	# scaling for publishing with setOverrideRCIN values between 1100 and 1900
	# neutral point is 1500
	pulse_width = value * 1000 + 1500 

	# Saturation
	if pulse_width > Vmax_mot:
		pulse_width = Vmax_mot
	if pulse_width < Vmin_mot:
		pulse_width = Vmin_mot

	return int(pulse_width)

def mapForceToPWM(Force, Voltage):
	return


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
	# This function replaces setservo for motor commands.
	# It overrides RC channels inputs and simulates motor controls.
	# In this case, each channel manages a group of motors not individually as servo set

	# Initialize message
	msg_override = OverrideRCIn()
	
	# Body force
	msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[0]--> surge	
	msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[1]--> sway
	msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave 
	msg_override.channels[3] = np.uint( channel_yaw)        # pulseCmd[3]--> roll
	msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[4]--> pitch
	msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[5]--> yaw
	
	# msg_override.channels[6] = 1500							# pulseCmd[6]--> light
	# msg_override.channels[7] = 1500							# pulseCmd[5]--> camera servo

	# Log override
	# rospy.loginfo(f"{msg_override.channels[0]} {msg_override.channels[1]} {msg_override.channels[2]} {msg_override.channels[3]} {msg_override.channels[4]} {msg_override.channels[5]}")

    # Publish the override message

	pub_msg_override.publish(msg_override)

# ----------- ROS ---------------

def subscriber():
	# Subscribers
	rospy.Subscriber("joy", Joy, joyCallback)
	rospy.Subscriber("cmd_vel", Twist, velCallback)
	rospy.Subscriber("mavros/imu/data", Imu, imuCallback)
	# rospy.Subscriber("mavros/imu/static_pressure", FluidPressure, depthPressureCallback)
	rospy.Subscriber("mavros/global_position/rel_alt", Float64, depthCallback)

	# /USV/mavros/global_position/rel_alt


if __name__ == '__main__':
	# Initial condition set as Disarmed
	armDisarm(False)

	# Initialize node
	rospy.init_node('Simple_PID_node', anonymous=False)

	# Publish PID signal to actual thruster
	pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10, tcp_nodelay = True)

	# Publish depth and orientation
	pub_depth = rospy.Publisher('/USV/depth/state', Float64, queue_size = 10, tcp_nodelay = True)
	# pub_angle_degre = rospy.Publisher('/USV/angle_degree', Twist, queue_size = 10, tcp_nodelay = True)
	pub_angle_rad = rospy.Publisher('/USV/angle_radian', Twist, queue_size = 10, tcp_nodelay = True)

	# Publish linear and angular velocity
	pub_angular_velocity = rospy.Publisher('/USV/angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
	pub_linear_velocity = rospy.Publisher('/USV/linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)

	# Publish commanded body force
	pub_tau_stb = rospy.Publisher("/USV/tau_stb", Float64MultiArray, queue_size=10, tcp_nodelay = True)

	# Call upon subcscriber
	subscriber()

	# Inital message
	rospy.loginfo("Manual mode - DISARMED - no control input from PID.")
	rospy.loginfo("Please ARMED before continue.")

	# Keep the node running
	rospy.spin()