import math
import os
import sys
import rclpy
import rospkg
import time
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist, Pose, Point  # Twistメッセージ型をインポート
from nav_msgs.msg import Odometry    # Odometryメッセージ型をインポート
from sensor_msgs.msg import LaserScan    # Odometryメッセージ型をインポート
from tf_transformations import euler_from_quaternion 

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


class HappyLidar(Node):  # 簡単な移動クラス
    def __init__(self):   # コンストラクタ
        super().__init__('happy_lidar_node')        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) 
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)    
        self.sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)   
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.scan  = LaserScan()
 
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0   
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()  # Twist メッセージ型インスタンスの生成
        self.set_vel(0.0, 0.0)  # 速度の初期化        
 
    def get_pose(self, msg):      # 姿勢を取得する
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        return x, y, yaw

    def print_lidar_info(self):
        # シミュレーションのPRDLIARは全周360[°] 計測可能。計測角度は-180[°]から180[°]。
        # ROSは右手系、進行方向x軸、左方向y軸、上方向がz軸（反時計まわりが正)。
        if len(self.scan.ranges) == 0:
            return
        self.get_logger().info(
            f'Angle [rad] min={self.scan.angle_min:3f} max={self.scan.angle_max:3f}')
        self.get_logger().info(
            f'Angle2 [rad] min={self.angle_min:3f} max={self.angle_max:3f}')
        # ROSの角度は[rad]。math.degresss関数でラジアンから°に変換している。
        self.get_logger().info(
            f'Angle [deg] increment={math.degrees(self.scan.angle_increment):.3f}')
        self.get_logger().info(
            f'Range [m] min={self.scan.range_min:.3f} max={self.scan.range_max:.3f}')
        print(f'len={len(self.scan.ranges)}')
        print(f'r[{0}]={self.scan.ranges[0]} ')
        print(f'r[{90}]={self.scan.ranges[90]} ')
        print(f'r[{180}]={self.scan.ranges[180]} ')
        print(f'r[{270}]={self.scan.ranges[270]} ')
        print(f'r[{359}]={self.scan.ranges[359]} ')

        #self.get_logger().info(
        #    f'  0[deg]={self.scan.ranges[0]:.3f}[m]  90[deg]={self.scan.ranges[90]:.3f}[m]')
        #self.get_logger().info(
        #    f' 180[deg]={self.scan.ranges[180]:.3f}[m] -90[deg]={self.scan.ranges[270]:.3f}[m]')
  
    def lidar_cb(self, msg):         # オドメトリのコールバック関数
        #self.scan = LaserScan()
        self.scan = msg
        #print(f'len(ranges)={len(msg.ranges)}')
        #print(self.scan)

        for i in range(len(msg.ranges)):
            #print(f'r[{i}]={msg.ranges[i]} ')
            #print(f'r[{i}]={self.scan.ranges[i]} ')
            if msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max:
                self.scan.ranges[i] = math.nan
            else:
                self.scan.ranges[i] = msg.ranges[i]
        
        #self.ranges          = msg.ranges
        self.angle_min       = msg.angle_min  # スキャンの開始角度[rad]
        self.angle_max       = msg.angle_max  # スキャンの終了角度[rad]
        self.angle_increment = msg.angle_increment  # 角度分解能[rad]
        self.range_min       = msg.range_min  # 最小検出距離[m] 
        self.range_max       = msg.range_max  # 最大検出距離[m]
      
    def odom_cb(self, msg):         # オドメトリのコールバック関数
        self.x, self.y, self.yaw = self.get_pose(msg)
        self.get_logger().info(
            f'x={self.x: .2f} y={self.y: .2f}[m] yaw={self.yaw: .2f}[rad/s]')     
    
    def set_vel(self, linear, angular):  # 速度を設定する
        self.vel.linear.x = linear   # [m/s]
        self.vel.angular.z = angular  # [rad/s]    
    
    def move_distance(self, dist):  # 指定した距離distを移動する
        error = 0.05  # 許容誤差 [m] 
        diff = dist - math.sqrt((self.x-self.x0)**2 + (self.y-self.y0)**2) 
        if math.fabs(diff) > error:
            self.set_vel(0.25, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def rotate_angle(self, angle):  # 指定した角度angleを回転する
        # このメソッドは間違っています．move_distanceを参考に完成させてください．
        self.set_vel(0.0, 0.25)
        return False

    def timer_callback(self):  # タイマーのコールバック関数
        self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ 
    
    #https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985
    #  Gazebo3Dモデルのロード
    def load_gazebo_models(self):
        #load_node = rclpy.create_node('load_node')
        door_reference_frame='world'
        door_pose = Pose(position=Point(x=1.1, y=-0.2, z=0.5))

        # 3Dモデルのパスを取得
        #model_path = rospkg.RosPack().get_path('happy_mini_turtlebot3_sim')+'/turtlebot3_gazebo/models/'
        #print(f'model_path={model_path}')
        model_path = '/home/ubuntu/airobot_ws/src/chapter2/happy_mini_turtlebot3_sim/turtlebot3_gazebo/models/'

        client = self.create_client(SpawnEntity, "/spawn_entity")

        self.get_logger().info("Connecting to `/spawn_entity` service...")
        if not client.service_is_ready():
            client.wait_for_service()
            self.get_logger().info("connected!")
        
        # Get path to the door
        sdf_file_path = os.path.join(
            get_package_share_directory("turtlebot3_gazebo"), "models",
            "door", "model.sdf")

        # Set data for request
        request = SpawnEntity.Request()
        request.name = 'door'
        request.xml = open(sdf_file_path, 'r').read()
        request.robot_namespace = ''
        request.reference_frame = 'world'
        request.initial_pose.position.x = 1.1
        request.initial_pose.position.y = -0.2
        request.initial_pose.position.z = 0.5

        self.get_logger().info("Sending service request to `/spawn_entity`")
        future =  client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('response: %r' % future.result())
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())

        #load_node.destroy_node()

    #  Gazebo 3Dモデルの削除
    def delete_gazebo_models(self):
        print('delete_gazebo_models')
        #delete_node = rclpy.create_node('delete_node')
        client = self.create_client(DeleteEntity, '/delete_entity')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not availabe. waiting...')

        request = DeleteEntity.Request()
        request.name = 'door'      
     
        self.get_logger().info("Sending service request to `/delete_entity`")
        future =  client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('response: %r' % future.result())
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())
        
        #delete_node.destroy_node()


    def happy_lidar(self): 
        duration = 0.1
        steps = 0

        #  Doorのロード
        print('load_gazebo_models')
        self.load_gazebo_models()


        while rclpy.ok():
            # ハンズオン１：ここに、何かに0.3[m]以内に近づいたら停止するコードを書く
            print(f'step={steps}')            

            print(f'r[{0}]={self.scan.ranges[0]} ')
            if self.scan.ranges[0] > 1.0:
                self.set_vel(0.2,0.0)

            # print(f'duration*steps={duration*steps}')
            if duration * steps == 10:
                print('call delete_gazebo_models ')
                self.delete_gazebo_models()    
            rclpy.spin_once(self)
            # self.print_lidar_info() # after spin_once
            time.sleep(duration)
            steps += 1

def main(args=None):  # main関数
    rclpy.init(args=args)
    node = HappyLidar()

    try:
        node.happy_lidar()
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')     
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
