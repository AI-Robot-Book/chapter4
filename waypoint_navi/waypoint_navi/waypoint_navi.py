#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, time
from math import pi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, \
  Point, Quaternion, Twist
import tf_transformations
from tf_transformations import euler_from_quaternion
from nav2_msgs.action import NavigateToPose


class WayPointNavi(Node):
  def __init__(self): 
    super().__init__('waypoint_navi')  # ノードの初期化
    self.wp_num = 0
    self.init_pos = [-2.0, -0.5]

    self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                               'initialpose',10)    


  def send_goal(self, pose): 
    while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
      self.get_logger().info("アクションサーバが起動するまで待つわ．")
    self.get_logger().info("No.{}({},{})に行きます．".format(self.wp_num+1,
                      pose.pose.position.x,pose.pose.position.y))
 
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = pose
      
    send_goal_future = self.nav_to_pose_client.send_goal_async(
                      goal_msg,feedback_callback=self.feedback_callback)
    rclpy.spin_until_future_complete(self, send_goal_future)
    self.goal_handle = send_goal_future.result()

    if not self.goal_handle.accepted:
      self.get_logger().error('No.{}は否認されました．'.format(self.wp_num+1))
      return

    self.get_logger().info('No.{}は承認されました．'.format(self.wp_num+1))
    self.result_future = self.goal_handle.get_result_async()
    self.result_future.add_done_callback(self.get_result_callback)    


  def get_result_callback(self, future): 
    if future.result().status == GoalStatus.STATUS_SUCCEEDED:
      self.get_logger().info('No.{}に着きました．'.format(str(self.wp_num+1)))
      self.wp_num += 1
    elif future.result().status == GoalStatus.STATUS_CANCELED:
      self.wp_num += 1

      
  def feedback_callback(self, msg):
    self.feedback = msg.feedback
    self.get_logger().info("残り{:.2f}[m]".format(self.feedback.distance_remaining))


  def cancel_goal(self):
    self.get_logger().info('No.{}をキャンセルします．'.format(self.wp_num+1))
    if self.result_future:
      future = self.goal_handle.cancel_goal_async()
      rclpy.spin_until_future_complete(self, future)


  def do_navigation(self):
    way_point = [[1.2, -1.5, pi/2], [1.0, 0.5, pi], [-4.0, 0.8, pi/2], \
      [-4.0, 3.9, pi],   [-6.5, 4.0, -pi/2], [-6.5, -3.0, pi/2], [999.9, 0.0, 0.0]]

    self.set_initial_pose()   # 初期位置の設定

    while rclpy.ok():
      if way_point[self.wp_num][0] == 999.9:       # 終了処理
        self.get_logger().info('ナビゲーションを終了します．') 
        sys.exit()
      elif self.wp_num == 3:  # 特定ののウェイポイントをキャンセルして次へ行く
        self.cancel_goal()

      pose = self.set_pose(way_point[self.wp_num]) # ウェイポイントの姿勢の設定
      self.send_goal(pose)  # ゴールの送信
      time.sleep(1)  # スリープ


  def set_initial_pose(self):
    initial_pose = Pose()
    initial_pose.position.x = self.init_pos[0]
    initial_pose.position.y = self.init_pos[1]
    initial_pose.orientation.x = 0.0
    initial_pose.orientation.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0
    msg = PoseWithCovarianceStamped()
    msg.pose.pose = initial_pose
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    self.get_logger().info('初期位置をパブリッシュします．')
    self.initial_pose_pub.publish(msg)


  def set_pose(self,wp_pose):
    pose = PoseStamped()
    pose.header.stamp = self.get_clock().now().to_msg()
    pose.header.frame_id = "map"
    pose.pose.position.x = wp_pose[0]
    pose.pose.position.y = wp_pose[1]
    q = tf_transformations.quaternion_from_euler(0, 0, wp_pose[2])
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose
      

def main(args=None):
  rclpy.init(args=args)
  waypoint_navi = WayPointNavi()
  waypoint_navi.do_navigation()
  rclpy.spin(waypoint_navi)
  waypoint.destroy_node()
  rclpy.shutdown()

    
if __name__ == '__main__':
    main()

