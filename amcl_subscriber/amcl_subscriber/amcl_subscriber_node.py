import rclpy
import tf_transformations
from rclpy.node import Node                                                                             
from std_msgs.msg import String                                                                         
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class AMCLSubscriber(Node):                                                                               
    def __init__(self):        # コンストラクタ
        super().__init__('amcl_subscriber_node')                                                          
        self.create_subscription(PoseWithCovarianceStamped,
                                                 'amcl_pose', self.amcl_cb, 10)                  
        self.create_subscription(Odometry,'odom',self.odom_cb, 10)                  
                                                                                                        

    def get_pose(self, msg):      # 姿勢の取得
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))
        return x, y, yaw


    def amcl_cb(self, msg):         # AMCLのコールバック関数
        x, y, yaw = self.get_pose(msg)
        self.get_logger().info("AMCL：x={:.2f} y={:.2f}[m] theta={:.2f}[rad/s]"
             .format(x, y, yaw))

    def odom_cb(self, msg):         # ODOMのコールバック関数
        x, y, yaw = self.get_pose(msg)
        self.get_logger().info("ODOM：x={:.2f} y={:.2f}[m] theta={:.2f}[rad/s]"
             .format(x, y, yaw))

        
def main(args=None):                                                                                    
    rclpy.init(args=args)                                                                               
    amcl_subscriber = AMCLSubscriber()                                                                      
    rclpy.spin(amcl_subscriber)                                                                           
    amcl_subscriber.destory_node()                                                                        
    rclpy.shutdown()                                                                                    
