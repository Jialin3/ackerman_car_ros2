import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_geometry_msgs import TransformStamped
import math
from visualization_msgs.msg import Marker



class pure_pursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.declare_parameter("waypoints_path", "/root/yahboomcar_ros2_ws/f1tenth_ws/src/pure_pursuit/racelines/test_track1.csv")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cloest_point_topic", "/cloest_point")
        self.declare_parameter("target_point_topic", "/target_point")
        self.declare_parameter("K_p", 0.5)
        self.declare_parameter("max_angle",0.78) # 0.78rad = 45度
        self.declare_parameter("lookahead_distance",0.2)
        self.declare_parameter("global_refFrame", "map")
        self.declare_parameter("car_refFrame", "base_footprint")

        self.waypoints_world_path = str(self.get_parameter("waypoints_path").value)
        self.drive_topic = str(self.get_parameter("drive_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cloest_point_topic = str(self.get_parameter("cloest_point_topic").value)
        self.target_point_topic = str(self.get_parameter("target_point_topic").value)
        self.K_p = float(self.get_parameter("K_p").value)
        self.max_angle = float(self.get_parameter("max_angle").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.global_refFrame = str(self.get_parameter("global_refFrame").value)
        self.car_refFrame = str(self.get_parameter("car_refFrame").value)

        self.load_waypoints(self.waypoints_world_path)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.car_trans = TransformStamped()
        self.car_pose = PoseStamped()
        self.target_point_source = PointStamped()
        self.target_point_source.header.frame_id = self.global_refFrame

        self.cloest_point_marker = Marker()
        self.cloest_point_marker.header.frame_id = self.global_refFrame
        self.cloest_point_marker.type = Marker.SPHERE
        self.cloest_point_marker.action = Marker.ADD
        self.cloest_point_marker.scale.x = 0.25
        self.cloest_point_marker.scale.y = 0.25
        self.cloest_point_marker.scale.z = 0.25
        self.cloest_point_marker.color.a = 1.0
        self.cloest_point_marker.color.b = 1.0
        self.cloest_point_marker.id = 1

        self.target_point_marker = Marker()
        self.target_point_marker.header.frame_id = self.global_refFrame
        self.target_point_marker.type = Marker.SPHERE
        self.target_point_marker.action = Marker.ADD
        self.target_point_marker.scale.x = 0.25
        self.target_point_marker.scale.y = 0.25
        self.target_point_marker.scale.z = 0.25
        self.target_point_marker.color.a = 1.0
        self.target_point_marker.color.r = 1.0
        self.target_point_marker.id = 1

        # publishers
        self.create_timer(0.1, self.timer_callback)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.cloest_point_pub = self.create_publisher(Marker,self.cloest_point_topic,1)
        self.target_point_pub = self.create_publisher(Marker,self.target_point_topic,1)

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)


    def timer_callback(self):
        self.waypoints_world_path = str(self.get_parameter("waypoints_path").value)
        self.K_p = float(self.get_parameter("K_p").value)
        self.transform_callback()

        self.car_pose.header = self.car_trans.header
        self.car_pose.pose.position.x = self.car_trans.transform.translation.x
        self.car_pose.pose.position.y = self.car_trans.transform.translation.y
        self.car_pose.pose.position.z = self.car_trans.transform.translation.z
        self.car_pose.pose.orientation.x = self.car_trans.transform.rotation.x
        self.car_pose.pose.orientation.y = self.car_trans.transform.rotation.y
        self.car_pose.pose.orientation.z = self.car_trans.transform.rotation.z
        self.car_pose.pose.orientation.w = self.car_trans.transform.rotation.w

        [y,L] = self.get_target()
        self.Pi_control(y,L)

    def Pi_control(self,y,L):
        angle_radian = self.K_p * 2 * y / pow(L,2)
        if angle_radian < -self.max_angle:
            angle_radian = -self.max_angle
        elif angle_radian > self.max_angle:
            angle_radian = self.max_angle
        driver_cmd = AckermannDriveStamped()
        driver_cmd.drive.steering_angle = angle_radian
        driver_cmd.drive.speed = 0.1
        self.drive_pub.publish(driver_cmd)                 
        
    def odom_callback(self, msgs:Odometry):
        self.car_cur_velocity = math.sqrt(pow(msgs.twist.twist.linear.x,2) + pow(msgs.twist.twist.linear.y,2))

    def get_target(self):
        car_cur = [0,0]
        car_cur[0] = self.car_pose.pose.position.x 
        car_cur[1] = self.car_pose.pose.position.y
        shortest_distance =  np.linalg.norm(self.waypoints[0]- car_cur)
        closet_idx = 0
        for i in range(len(self.waypoints)):
            if np.linalg.norm(self.waypoints[i]- car_cur) < shortest_distance:
                shortest_distance = np.linalg.norm(self.waypoints[i]- car_cur)
                closet_idx = i
        self.cloest_point_marker.pose.position.x = self.waypoints[closet_idx,0]
        self.cloest_point_marker.pose.position.y = self.waypoints[closet_idx,1]
        self.cloest_point_pub.publish(self.cloest_point_marker)

        longest_distance = 0
        target_idx = closet_idx
        for i in range(closet_idx,len(self.waypoints)):
            if np.linalg.norm(self.waypoints[i]- car_cur) <= self.lookahead_distance and np.linalg.norm(self.waypoints[i]- car_cur) > longest_distance:
                longest_distance = np.linalg.norm(self.waypoints[i]- car_cur)
                target_idx = i
        self.target_point_marker.pose.position.x = self.waypoints[target_idx,0]
        self.target_point_marker.pose.position.y = self.waypoints[target_idx,1]
        self.target_point_pub.publish(self.target_point_marker)
        self.target_point_source.point.x = self.waypoints[target_idx,0]
        self.target_point_source.point.y = self.waypoints[target_idx,1]
        self.target_point_source.point.z = 0.0
        target_point_car_frame = self.buffer.transform(self.target_point_source,"base_footprint")
        return target_point_car_frame.point.y, math.sqrt(pow(target_point_car_frame.point.x,2) + pow(target_point_car_frame.point.y,2))              
                  
    def transform_callback(self):
        try:
            self.now = rclpy.time.Time()
            self.car_trans = self.buffer.lookup_transform(self.global_refFrame,self.car_refFrame,self.now)
        except TransformException as ex:
             self.get_logger().info(f'Could not transform : {ex}')

    def load_waypoints(self, file_path):
        self.waypoints = np.genfromtxt(file_path, delimiter=",")[:, :2]
        self.waypoints_velocities = np.genfromtxt(file_path, delimiter=",")[:, 2]
      
def main(args=None):
    rclpy.init(args=args)
    print("pure pursuit Initialized")
    pure_pursuit_node = pure_pursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


        
        



            









            


        

