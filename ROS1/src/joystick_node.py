#!/usr/bin/env python3

import triad_openvr, time
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
import math
from tf.transformations import quaternion_from_euler, quaternion_multiply


class JoystickNode:
    def __init__(self):
        rospy.init_node('joystick_node', anonymous=True)

        self.position_publisher = rospy.Publisher('/dxl_input/pos_right', PoseStamped, queue_size=10)
        self.gripper_publisher = rospy.Publisher('joystick_gripper', PointStamped, queue_size=10)
        self.marker_publisher = rospy.Publisher('joystick_marker', Marker, queue_size=10)

        self.v = triad_openvr.triad_openvr()

        self.space_scale = 1.0

        self.controller_name = "controller_1"
        self.rate = rospy.Rate(50)

        self.pose_msg = PoseStamped()
        self.gripper_msg = PointStamped()

        self.initial_position = None
        
        #self.robot_position = rospy.wait_for_message("/cartesian/gripper_right_grasping_frame/current_reference", PoseStamped, timeout=5).pose.position

    def rotate_quaternion(self, q_in):
        q_offset = quaternion_from_euler(-math.pi/2, -math.pi/4, 3*math.pi/4)
        q_out = quaternion_multiply(q_offset, q_in)
        return q_out

    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                euler_pose = self.v.devices[self.controller_name].get_pose_quaternion()
                controller_inputs = self.v.devices[self.controller_name].get_controller_inputs()

                if euler_pose is None:
                    rospy.logwarn("Joystick is not tracking")
                    # TODO: Trigger a reset to the home position or do something else
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
                    reset_button = controller_inputs.get('menu_button', 0)

                    pose_x = euler_pose[0]
                    pose_y = euler_pose[1]
                    pose_z = euler_pose[2]
                    q_x = euler_pose[3]
                    q_y = euler_pose[4]
                    q_z = euler_pose[5]
                    q_w = euler_pose[6]

                    q_in = [q_x, q_y, q_z, q_w]
                    q_corrected = self.rotate_quaternion(q_in)

                    if reset_button:
                        self.initial_position = [pose_x, pose_y, pose_z]
                        rospy.loginfo("Initial position reset to: x=%f, y=%f, z=%f" %
                                      (pose_x, pose_y, pose_z))

                    if trackpad_pressed:
                        if self.initial_position is None:
                            self.initial_position = [pose_x, pose_y, pose_z]
                            rospy.loginfo("Initial position set to: x=%f, y=%f, z=%f" %
                                          (pose_x, pose_y, pose_z))

                        relative_x = pose_x - self.initial_position[0]
                        relative_y = pose_y - self.initial_position[1]
                        relative_z = pose_z - self.initial_position[2]


                        self.pose_msg.header.frame_id = "ci/world"
                        self.pose_msg.header.stamp = rospy.Time.now()
                        self.pose_msg.pose.position.x = relative_x * self.space_scale #+ self.robot_position.x
                        self.pose_msg.pose.position.y = relative_y * self.space_scale #+ self.robot_position.y
                        self.pose_msg.pose.position.z = relative_z * self.space_scale #+ self.robot_position.z

                        self.pose_msg.pose.orientation.x = q_corrected[0]
                        self.pose_msg.pose.orientation.y = q_corrected[1]
                        self.pose_msg.pose.orientation.z = q_corrected[2]
                        self.pose_msg.pose.orientation.w = q_corrected[3]

                        self.position_publisher.publish(self.pose_msg)

                        self.gripper_msg.header = self.pose_msg.header
                        self.gripper_msg.point.x = trigger_value
                        self.gripper_publisher.publish(self.gripper_msg)

                        marker = Marker()
                        marker.header = self.pose_msg.header
                        marker.type = Marker.ARROW
                        marker.action = Marker.ADD
                        marker.scale.x = 0.5
                        marker.scale.y = 0.3
                        marker.scale.z = 0.3
                        marker.color.a = 1.0
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        marker.pose = self.pose_msg.pose
                        self.marker_publisher.publish(marker)

            except Exception as e:
                rospy.logerr("Controller is not linked or is out of range")
                rospy.logerr(e)
                return

            self.rate.sleep()


if __name__ == '__main__':
    joystick_node = JoystickNode()
    joystick_node.main_loop()
