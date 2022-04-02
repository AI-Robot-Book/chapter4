import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# 速度指令値のトピック cmd_vel をサブスクライブして端末に表示するたけの簡単なクラス
class HappySubscriber(Node):
    def __init__(self):  # コンストラクタ
        super().__init__('happy_subscriber_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel',
                                                     self.callback, 10)

    def callback(self, Twist):  # コールバック関数．端末に並進と角速度を表示する
        self.get_logger().info(f'並進速度={Twist.linear.x} 角速度={Twist.angular.z}')


def main():  # main関数
    rclpy.init()
    node = HappySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+CLが押されました．')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    rclpy.shutdown()
