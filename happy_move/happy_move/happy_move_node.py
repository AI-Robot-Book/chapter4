import math
import sys
import rclpy
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist  # Twist メッセージ型をインポート
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class HappyMove(Node):  # キー操作により速度指令値をパブリッシュするクラス
    def __init__(self):   # コンストラクタ
        super().__init__('happy_move_node')        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)   
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()  # Twist メッセージ型インスタンスの生成
        self.vel.linear.x = 0.0   # [m/s]
        self.vel.angular.z = 0.0  # [rad/s]
        self.distance = 2.0  # [m]
        self.angle = math.pi/2  # [rad]
 
    def get_pose(self, msg):      # 姿勢の取得
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        return x, y, yaw
  
    def odom_cb(self, msg):         # ODOMのコールバック関数
        self.x, self.y, self.yaw = self.get_pose(msg)
        self.get_logger().info(
            f'x={self.x: .2f} y={self.y: .2f}[m] yaw={self.yaw: .2f}[rad/s]')     
    
    def set_vel(self, linear, angular):
        self.vel.linear.x = linear   # [m/s]
        self.vel.angular.z = angular  # [rad/s]    
    
    def move_distance(self, distance):
        error = 0.05  # [m] 
        diff = distance - math.sqrt((self.x - self.x0) ** 2 + (self.y - self.y0) ** 2)
        if diff > error:
            self.set_vel(0.25, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def rotate_angle(self, angle):
        error = 0.1
        diff = 1.0  # change this line
        if diff > error:
            self.set_vel(0.0, 0.25)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def timer_callback(self):  # タイマーのコールバック関数
        self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ
 
    def happy_move(self):
        state = 0
        print('moving...')
        while rclpy.ok():
            if state == 0:
                if self.move_distance(self.distance):
                    state = 1
                    print('rotating...')
            elif state == 1:                
                if self.rotate_angle(self.angle):
                    state = 2
                    print('end')
            elif state == 2:
                break
            else:
                print('Invalid state')
            rclpy.spin_once(self)


def main(args=None):  # main関数
    rclpy.init(args=args)
    node = HappyMove()

    try:
        node.happy_move()
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')     
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
