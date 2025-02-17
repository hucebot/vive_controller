#!/usr/bin/env python3

import triad_openvr, time
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
import math
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse, euler_from_quaternion

class KalmanFilter:
    def __init__(self, dt, state_dim, measurement_dim):
        self.dt = dt
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim

        self.A = np.eye(state_dim)
        self.H = np.eye(measurement_dim, state_dim)
        self.Q = np.eye(state_dim) * 0.01
        self.R = np.eye(measurement_dim) * 0.1
        self.P = np.eye(state_dim)
        self.x = np.zeros((state_dim, 1))

    def predict(self):
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)
        return self.x

class JoystickNode:
    def __init__(self):
        rospy.init_node('joystick_node', anonymous=True)

        self.position_publisher = rospy.Publisher('/dxl_input/pos_right', PoseStamped, queue_size=10)
        self.gripper_publisher = rospy.Publisher('joystick_gripper', PointStamped, queue_size=10)
        self.marker_publisher = rospy.Publisher('joystick_marker', Marker, queue_size=10)

        self.v = triad_openvr.triad_openvr()
        self.v.print_discovered_objects()

        self.space_scale = 1.0
        self.controller_name = "controller_1"
        self.rate = rospy.Rate(50)

        self.pose_msg = PoseStamped()
        self.gripper_msg = PointStamped()

        self.initial_position = None
        self.initial_orientation = None

        # Inicializar el filtro de Kalman
        self.dt = 1.0 / 50.0
        self.state_dim = 6
        self.measurement_dim = 6
        self.kf = KalmanFilter(self.dt, self.state_dim, self.measurement_dim)

    def publish_axes_marker(self, frame_id, pose, namespace="joystick_axes"):
        axis_length = 0.2
        axis_diameter = 0.015

        def make_arrow_marker(id, color, orientation_offset):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = rospy.Time.now()
            m.ns = namespace
            m.id = id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = axis_length
            m.scale.y = axis_diameter 
            m.scale.z = axis_diameter
            m.color.a = 1.0
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.pose.position.x = pose.position.x
            m.pose.position.y = pose.position.y
            m.pose.position.z = pose.position.z
            
            q_pose = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            q_offset = orientation_offset
            q_final = quaternion_multiply(q_pose, q_offset)

            m.pose.orientation.x = q_final[0]
            m.pose.orientation.y = q_final[1]
            m.pose.orientation.z = q_final[2]
            m.pose.orientation.w = q_final[3]

            return m

        q_id = [0, 0, 0, 1]
        q_y  = quaternion_from_euler(0, 0, math.pi/2)
        q_z  = quaternion_from_euler(0, -math.pi/2, 0) 

        marker_x = make_arrow_marker(0, (1.0, 0.0, 0.0), q_id)
        marker_y = make_arrow_marker(1, (0.0, 1.0, 0.0), q_y)
        marker_z = make_arrow_marker(2, (0.0, 0.0, 1.0), q_z)

        self.marker_publisher.publish(marker_x)
        self.marker_publisher.publish(marker_y)
        self.marker_publisher.publish(marker_z)

    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                euler_pose = self.v.devices[self.controller_name].get_pose_euler()
                controller_inputs = self.v.devices[self.controller_name].get_controller_inputs()

                if euler_pose is None:
                    rospy.logwarn("Joystick is not tracking")
                    self.pose_msg.header.frame_id = "ci/world"
                    self.pose_msg.header.stamp = rospy.Time.now()
                    self.pose_msg.pose.position.x = 0
                    self.pose_msg.pose.position.y = 0
                    self.pose_msg.pose.position.z = 0
                    self.pose_msg.pose.orientation.x = 0
                    self.pose_msg.pose.orientation.y = 0
                    self.pose_msg.pose.orientation.z = 0
                    self.pose_msg.pose.orientation.w = 1
                    self.position_publisher.publish(self.pose_msg)
                    self.initial_position = None

                if controller_inputs is not None and euler_pose is not None:
                    trigger_value = controller_inputs.get('trigger', 0)
                    trackpad_pressed = controller_inputs.get('trackpad_pressed', 0)
                    trackpad_touched = controller_inputs.get('trackpad_touched', 0)
                    reset_button = controller_inputs.get('menu_button', 0)

                    if reset_button:
                        rospy.loginfo("Resetting initial position")
                        self.initial_position = euler_pose[:3]
                        self.initial_orientation = quaternion_from_euler(
                            math.radians(euler_pose[3]),
                            math.radians(euler_pose[5]),
                            math.radians(euler_pose[4])
                        )

                    if self.initial_position is None: 
                        self.initial_position = euler_pose[:3]
                        self.initial_orientation = quaternion_from_euler(
                            math.radians(euler_pose[3]),
                            math.radians(euler_pose[5]),
                            math.radians(euler_pose[4])
                        )

                    pose_x = -round(euler_pose[0] - self.initial_position[0], 2)
                    pose_y = round(euler_pose[2] - self.initial_position[2], 2)
                    pose_z = round(euler_pose[1] - self.initial_position[1], 2)

                    q_current = quaternion_from_euler(
                        math.radians(euler_pose[3]),
                        math.radians(euler_pose[5]),
                        math.radians(euler_pose[4])
                    )
                    
                    q0_inv = quaternion_inverse(self.initial_orientation)
                    q_rel = quaternion_multiply(q0_inv, q_current)
                    q_rel = (q_rel[0], q_rel[1], -q_rel[2], q_rel[3]) 

                    r_rel, p_rel, y_rel = euler_from_quaternion(q_rel)

                    roll = math.degrees(r_rel)
                    pitch = math.degrees(p_rel)
                    yaw = math.degrees(y_rel)

                    measurement = np.array([[pose_x], [pose_y], [pose_z], [roll], [pitch], [yaw]])

                    self.kf.predict()
                    filtered_state = self.kf.update(measurement)

                    filtered_pose_x = filtered_state[0, 0]
                    filtered_pose_y = filtered_state[1, 0]
                    filtered_pose_z = filtered_state[2, 0]
                    filtered_roll = filtered_state[3, 0]
                    filtered_pitch = filtered_state[4, 0]
                    filtered_yaw = filtered_state[5, 0]

                    filtered_q = quaternion_from_euler(
                        math.radians(filtered_roll),
                        math.radians(filtered_pitch),
                        math.radians(filtered_yaw)
                    )

                    if trackpad_pressed:
                        self.pose_msg.header.frame_id = "ci/world"
                        self.pose_msg.header.stamp = rospy.Time.now()
                        self.pose_msg.pose.position.x = filtered_pose_y
                        self.pose_msg.pose.position.y = -filtered_pose_x
                        self.pose_msg.pose.position.z = filtered_pose_z
                        self.pose_msg.pose.orientation.x = filtered_q[0]
                        self.pose_msg.pose.orientation.y = filtered_q[1]
                        self.pose_msg.pose.orientation.z = filtered_q[2]
                        self.pose_msg.pose.orientation.w = filtered_q[3]
                        self.position_publisher.publish(self.pose_msg)

                    self.publish_axes_marker("ci/world", self.pose_msg.pose)

            except Exception as e:
                rospy.logerr("Controller is not linked or is out of range")
                rospy.logerr(e)
                return

            self.rate.sleep()


if __name__ == '__main__':
    joystick_node = JoystickNode()
    joystick_node.main_loop()
