#!/usr/bin/env python3
import math
import numpy as np
import time
import rospy
from mavros_msgs.msg import OverrideRCIn, State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray


# Set reference depths and orientation
ref_x = 0.0
ref_y = 0.0
ref_z = -0.5
ref_roll = 0.0
ref_pitch = 0.0
ref_yaw = 0.0

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

        # Compute proportional error
        error = self.setpoint - feedback_value

        # Compute error derivative
        derivative = ((error - self.last_error) / elapsed_time) if elapsed_time > 0 else 0

        # Compute integral act
        self.integral_act += error * elapsed_time
        
        # Compute tau_cmd
        tau_cmd = (self.Kp * error) + (self.Ki * self.integral_act) + (self.Kd * derivative)

        # Reset time 
        self.init_time = current_time
        self.proportional_error = error
        return tau_cmd

def compute_control(msg):
    # Translation (Euler)
    feedback_x = msg.pose.position.x
    feedback_y = msg.pose.position.y
    feedback_z = -msg.pose.position.z 
    
    # Orientation (Quaternion)
    feedback_orientation = np.degrees(tf.transformations.euler_from_quaternion(
        [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))
    feedback_pitch = feedback_orientation[0]
    feedback_roll = feedback_orientation[1]
    feedback_yaw = feedback_orientation[2]
    

    # Define PID for depth and yaw (TUNING AREA)
    x_PID = PID(setpoint=ref_x, Kp=1.0, Ki=0.1, Kd=0.05)
    y_PID = PID(setpoint=ref_y, Kp=1.0, Ki=0.1, Kd=0.05)
    z_PID = PID(setpoint=ref_z, Kp=1.0, Ki=0.1, Kd=0.05)
    roll_PID = PID(setpoint=np.deg2rad(ref_roll), Kp=0.4, Ki=0.09, Kd=0.4)
    pitch_PID = PID(setpoint=np.deg2rad(ref_pitch), Kp=0.4, Ki=0.09, Kd=0.4)
    yaw_PID = PID(setpoint=np.deg2rad(ref_yaw), Kp=0.4, Ki=0.09, Kd=0.4)

    # Compute outputs
    surge_control = x_PID.compute(feedback_x)
    sway_control = y_PID.compute(feedback_y)
    depth_control = z_PID.compute(feedback_z)
    roll_control = roll_PID.compute(np.deg2rad(feedback_roll))
    pitch_control = pitch_PID.compute(np.deg2rad(feedback_pitch))
    yaw_control = yaw_PID.compute(np.deg2rad(feedback_yaw))
    Tau_cmd = np.array([surge_control, sway_control, depth_control, roll_control, pitch_control, yaw_control])

    # Publish commanded thruster output (NEED TO ASK IF WE DON'T NEED THE THRUSTER ALLOCATION MATRIX)
    Tau_cmd_msg = Float64MultiArray()
    Tau_cmd_msg.data = Tau_cmd
    Tau_cmd_publisher.publish(Tau_cmd_msg)
    
    # tau_cmd to MAVROS
    send_rc_control(surge_control, sway_control, depth_control, pitch_control, roll_control, yaw_control)

def send_rc_control(surge, sway, depth, roll, pitch, yaw):
    # RC channels message
    # With saturation implemented (MIN/MAX = 1100/1900)
    # RC CONTROL MAPPING STILL NEEDS TO BE CONFIGURED
    control = OverrideRCIn()
    control.channels[0] = np.clip(1500 + np.int(surge * 10), 1100, 1900)
    control.channels[1] = np.clip(1500 + np.int(sway * 10), 1100, 1900)
    control.channels[2] = np.clip(1500 + np.int(depth * 10), 1100, 1900)
    control.channels[3] = np.clip(1500 + np.int(pitch * 10), 1100, 1900)
    control.channels[4] = np.clip(1500 + np.int(roll * 10), 1100, 1900)
    control.channels[5] = np.clip(1500 + np.int(yaw * 10), 1100, 1900)
    control.channels[6] = 1500
    control.channels[7] = 1500

    # Publish RC Control to MAVROS
    control_publisher.publish(control)

def subscriber():
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, compute_control)

if __name__ == "__main__":
    rospy.init_node("bluerov2_depth_controller")
    control_publisher = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    Tau_cmd_publisher = rospy.Publisher("/Tau_cmd", Float64MultiArray, queue_size=10)

    subscriber()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
