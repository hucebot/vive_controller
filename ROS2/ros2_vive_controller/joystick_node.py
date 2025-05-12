import rclpy
from rclpy.node import Node
import math
import yaml
from pathlib import Path
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply
from openvr_class.openvr_class import triad_openvr
from OneEuroFilter import OneEuroFilter


def read_yaml(path):
    with open(path, 'r') as stream:
        return yaml.safe_load(stream)


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        config_file = Path(__file__).parent.parent / 'config' / 'config.yaml'
        configs = read_yaml(str(config_file))

        self.robot_type = configs['general']['robot']
        self.v = triad_openvr(str(config_file))
        self.v.wait_for_n_tracking_references(3)
        self.v.reorder_tracking_references('LHB-DFA5BD2C')
        self.v.reindex_tracking_references()
        self.v.print_discovered_objects()

        self.controllers = self.v.return_controller_serials()
        self.publish_markers = configs['general']['publish_markers']
        self.use_left = configs['general']['use_left_controller']
        self.use_right = configs['general']['use_right_controller']
        rate_hz = configs['general']['rate']
        self.linear_scale = configs['general']['linear_scale']
        self.angular_scale = configs['general']['angular_scale']
        self.move_base = configs['general']['move_base']

        self.reset_sub = self.create_subscription(
            Bool,
            configs['general']['reset_position_topic'],
            self.reset_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        if self.use_right:
            rs = configs['htc_vive']['controller_1']['serial']
            self.right_name = self.controllers[rs]
            if self.robot_type == 'talos':
                pos_topic = configs['general']['talos_position_topic']
                grip_topic = configs['general']['talos_gripper_topic']
            else:
                pos_topic = configs['general']['right_position_topic']
                grip_topic = configs['general']['right_gripper_topic']
            self.pub_pos_right = self.create_publisher(PoseStamped, pos_topic, 10)
            self.pub_grip_right = self.create_publisher(PointStamped, grip_topic, 10)
            if self.publish_markers:
                self.marker_pub_right = self.create_publisher(Marker, configs['general']['right_marker_topic'], 10)
                self.aux_marker_pub_right = self.create_publisher(Marker, '/auxiliar_marker_right', 10)
            if self.move_base:
                self.pub_mb_ang = self.create_publisher(Float32, configs['general']['move_base_angular_topic'], 10)

            self.right_trigger = False
            self.right_ref = None
            self.right_cum = [0.0, 0.0, 0.0]
            self.right_init_ori = None

        if self.use_left:
            ls = configs['htc_vive']['controller_2']['serial']
            self.left_name = self.controllers[ls]
            self.pub_pos_left = self.create_publisher(PoseStamped, configs['general']['left_position_topic'], 10)
            self.pub_grip_left = self.create_publisher(PointStamped, configs['general']['left_gripper_topic'], 10)
            if self.publish_markers:
                self.marker_pub_left = self.create_publisher(Marker, configs['general']['left_marker_topic'], 10)
            if self.move_base:
                self.pub_mb_lin_x = self.create_publisher(Float32, configs['general']['move_base_linear_x_topic'], 10)
                self.pub_mb_lin_y = self.create_publisher(Float32, configs['general']['move_base_linear_y_topic'], 10)

            self.left_trigger = False
            self.left_ref = None
            self.left_cum = [0.0, 0.0, 0.0]
            self.left_init_ori = None

        self.right_filter = self._make_filters(configs)
        self.left_filter = self._make_filters(configs)

        self.workspace_limit = configs['general']['workspace_limit']
        ws = configs['workspace']
        self.x_max, self.x_min = ws['x_max'], ws['x_min']
        self.y_max, self.y_min = ws['y_max'], ws['y_min']
        self.z_max, self.z_min = ws['z_max'], ws['z_min']
        if self.publish_markers:
            self.pub_ws = self.create_publisher(Marker, 'workspace_bbox_marker', 10)
            self.pub_act = self.create_publisher(Marker, 'actual_pose_marker', 10)

        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _make_filters(self, configs):
        cfg = {
            'freq': configs['general']['rate'],
            'mincutoff': 1.0,
            'beta': 0.1,
            'dcutoff': 1.0
        }
        return {
            'x': OneEuroFilter(**cfg), 'y': OneEuroFilter(**cfg), 'z': OneEuroFilter(**cfg),
            'qx': OneEuroFilter(**cfg), 'qy': OneEuroFilter(**cfg),
            'qz': OneEuroFilter(**cfg), 'qw': OneEuroFilter(**cfg)
        }

    def reset_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('resetting controller state')
            self.right_cum = [0.0, 0.0, 0.0]
            ps = PoseStamped()
            ps.header.frame_id = 'ci/world'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.orientation.w = 1.0
            self.pub_pos_right.publish(ps)

    def publish_axes(self, frame_id, pose, pub):
        length, diam = 0.2, 0.015
        offs_id = quaternion_from_euler(0,0,0)
        offs_y = quaternion_from_euler(0,0,math.pi/2)
        offs_z = quaternion_from_euler(0,-math.pi/2,0)
        for idx, (col, off) in enumerate([((1,0,0), offs_id), ((0,1,0), offs_y), ((0,0,1), offs_z)]):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'joystick_axes'
            m.id = idx
            m.type = Marker.ARROW
            m.scale.x = length
            m.scale.y = diam
            m.scale.z = diam
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = col
            m.pose = pose
            qf = quaternion_multiply([
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w
            ], off)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qf
            pub.publish(m)
        if hasattr(self, 'aux_marker_pub_right'):
            a = Marker()
            a.header.frame_id = frame_id
            a.header.stamp = self.get_clock().now().to_msg()
            a.ns = 'auxiliar'
            a.id = 0
            a.type = Marker.ARROW
            a.scale.x, a.scale.y, a.scale.z = 0.1, 0.02, 0.02
            a.color.a = 1.0
            a.color.r = a.color.g = a.color.b = 1.0
            a.pose = pose
            self.aux_marker_pub_right.publish(a)

    def publish_workspace(self):
        m = Marker()
        m.header.frame_id = 'ci/world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'workspace'
        m.id = 1000
        m.type = Marker.CUBE
        m.scale.x = self.x_max - self.x_min
        m.scale.y = self.y_max - self.y_min
        m.scale.z = self.z_max - self.z_min
        m.pose.position.x = (self.x_max + self.x_min) / 2.0
        m.pose.position.y = (self.y_max + self.y_min) / 2.0
        m.pose.position.z = (self.z_max + self.z_min) / 2.0
        m.pose.orientation.w = 1.0
        m.color.g = 1.0
        m.color.a = 0.1
        self.pub_ws.publish(m)

    def parse_device(self, dev, side, pos_pub, grip_pub, marker_pub, filt):
        pq = dev.get_pose_quaternion()
        ci = dev.get_controller_inputs()
        if not pq or ci is None:
            return
        px, py, pz, qx, qy, qz, qw = *pq[:3], *pq[3:7]
        if (px < self.x_min + self.workspace_limit or px > self.x_max - self.workspace_limit or
            py < self.y_min + self.workspace_limit or py > self.y_max - self.workspace_limit or
            pz < self.z_min + self.workspace_limit or pz > self.z_max - self.workspace_limit):
            dev.trigger_haptic_pulse(1000, 0)
            return
        trig = ci.get('trigger', 0)
        grip_btn = ci.get('grip_button', 0)
        menu_btn = ci.get('menu_button', 0)
        if self.publish_markers:
            ap = Marker()
            ap.header.frame_id = 'ci/world'
            ap.header.stamp = self.get_clock().now().to_msg()
            ap.ns = 'actual_pose'
            ap.id = 0
            ap.type = Marker.SPHERE
            ap.scale.x = ap.scale.y = ap.scale.z = 0.05
            ap.pose.position.x, ap.pose.position.y, ap.pose.position.z = px, py, pz
            ap.pose.orientation.x,ap.pose.orientation.y,ap.pose.orientation.z,ap.pose.orientation.w = qx,qy,qz,qw
            ap.color.r = 1.0
            ap.color.a = 1.0
            self.publish_axes('ci/world', ap.pose, self.pub_act)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'ci/world'
            t.child_frame_id = 'vive_raw'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = px, py, pz
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
            self.tf_broadcaster.sendTransform(t)
        if side == 'right' and self.use_right:
            if self.right_init_ori is None:
                self.right_init_ori = [qx,qy,qz,qw]
            if grip_btn:
                self.right_init_ori = [qx,qy,qz,qw]
            if trig < 0.5:
                if self.right_trigger:
                    self.right_trigger = False
                    self.right_cum[0] += px - self.right_ref[0]
                    self.right_cum[1] += py - self.right_ref[1]
                    self.right_cum[2] += pz - self.right_ref[2]
            else:
                if not self.right_trigger:
                    self.right_trigger = True
                    self.right_ref = [px,py,pz]
                if ci.get('trackpad_pressed') and ci.get('trackpad_touched') and self.move_base:
                    ang = ci.get('trackpad_x',0) * self.angular_scale
                    self.pub_mb_ang.publish(Float32(data=ang))
                dx = px - self.right_ref[0]
                out = [self.right_cum[i] + d for i,d in enumerate((dx, py-self.right_ref[1], pz-self.right_ref[2]))]
                fx = filt['x'](out[0], self.get_clock().now().nanoseconds*1e-9)
                fy = filt['y'](out[1], self.get_clock().now().nanoseconds*1e-9)
                fz = filt['z'](out[2], self.get_clock().now().nanoseconds*1e-9)
                ps = PoseStamped()
                ps.header.frame_id = 'ci/world'
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = fx * self.linear_scale
                ps.pose.position.y = fy * self.linear_scale
                ps.pose.position.z = fz * self.linear_scale
                ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = qx,qy,qz,qw
                pos_pub.publish(ps)
                gr = PointStamped()
                gr.header = ps.header
                gr.point.x = abs(menu_btn)
                grip_pub.publish(gr)
                if self.publish_markers:
                    self.publish_axes('ci/world', ps.pose, marker_pub)
        if side == 'left' and self.use_left:
            if self.left_init_ori is None:
                self.left_init_ori = [qx,qy,qz,qw]
            if grip_btn:
                self.left_init_ori = [qx,qy,qz,qw]
            if trig < 0.5:
                if self.left_trigger:
                    self.left_trigger = False
                    self.left_cum[0] += px - self.left_ref[0]
                    self.left_cum[1] += py - self.left_ref[1]
                    self.left_cum[2] += pz - self.left_ref[2]
            else:
                if not self.left_trigger:
                    self.left_trigger = True
                    self.left_ref = [px,py,pz]
                if ci.get('trackpad_pressed') and ci.get('trackpad_touched') and self.move_base:
                    linx = ci.get('trackpad_y',0) * self.linear_scale
                    self.pub_mb_lin_x.publish(Float32(data=linx))
                out = [self.left_cum[i] + (val - self.left_ref[i]) for i,val in enumerate((px,py,pz))]
                fx = filt['x'](out[0], self.get_clock().now().nanoseconds*1e-9)
                fy = filt['y'](out[1], self.get_clock().now().nanoseconds*1e-9)
                fz = filt['z'](out[2], self.get_clock().now().nanoseconds*1e-9)
                ps = PoseStamped()
                ps.header.frame_id = 'ci/world'
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = fx * self.linear_scale
                ps.pose.position.y = fy * self.linear_scale
                ps.pose.position.z = fz * self.linear_scale
                ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = qx,qy,qz,qw
                pos_pub.publish(ps)
                gr = PointStamped()
                gr.header = ps.header
                gr.point.x = abs(menu_btn)
                grip_pub.publish(gr)
                if self.publish_markers:
                    self.publish_axes('ci/world', ps.pose, marker_pub)

    def timer_callback(self):
        if self.use_right:
            self.parse_device(
                self.v.devices[self.right_name], 'right',
                self.pub_pos_right, self.pub_grip_right,
                getattr(self, 'marker_pub_right', None), self.right_filter
            )
        if self.use_left:
            self.parse_device(
                self.v.devices[self.left_name], 'left',
                self.pub_pos_left, self.pub_grip_left,
                getattr(self, 'marker_pub_left', None), self.left_filter
            )
        if self.publish_markers:
            self.publish_workspace()


def main():
    rclpy.init()
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
