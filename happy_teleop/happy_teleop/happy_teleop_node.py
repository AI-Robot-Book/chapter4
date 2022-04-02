import rclpy                        
from rclpy.node import Node          
from geometry_msgs.msg import Twist  # Twist メッセージ型をインポート


class HappyTeleop(Node):  # キー操作により速度指令値をパブリッシュするクラス
    def __init__(self):   # コンストラクタ
        super().__init__('happy_teleop_node')        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.vel = Twist()  # Twist メッセージ型インスタンスの生成
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

    def timer_callback(self):  # タイマーのコールバック関数
        key = input('f, b, r, lキー入力後にEnterキーを押下 <<')  # キー取得
        # キーの値により並進速度や角速度を変更
        if key == 'f':
            self.vel.linear.x += 0.1
        elif key == 'b':
            self.vel.linear.x -= 0.1
        elif key == 'l':
            self.vel.angular.z += 0.1
        elif key == 'r':
            self.vel.angular.z -= 0.1
        elif key == 's':
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        else:
            print('入力キーが違います．')
        self.publisher.publish(self.vel)  # 速度指令メッセージのパブリッシュ
        self.get_logger().info(f'並進速度={self.vel.linear.x} 角速度={self.vel.angular.z}')


def main():  # main関数
    rclpy.init()
    node = HappyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    rclpy.shutdown()
