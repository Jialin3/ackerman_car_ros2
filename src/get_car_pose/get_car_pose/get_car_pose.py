import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import TransformStamped
from geometry_msgs.msg import  PoseStamped

class get_car_pose(Node):
    def __init__(self):
        super().__init__("get_car_pose_node")

        self.declare_parameter("global_refFrame", "odom")
        self.declare_parameter("car_refFrame", "base_link")
        
        self.global_refFrame = str(self.get_parameter("global_refFrame").value)
        self.car_refFrame = str(self.get_parameter("car_refFrame").value)


        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)

        self.create_timer(1.0, self.time_callback)
        self.pub_pose = self.create_publisher(PoseStamped,"/car_pose",1)


    def time_callback(self):
        self.global_refFrame = str(self.get_parameter("global_refFrame").value)
        self.car_refFrame = str(self.get_parameter("car_refFrame").value)
        car_pose = PoseStamped()
        t = self.get_clock().now()
        car_pose.header.frame_id = self.global_refFrame
        car_pose.header.stamp = t.to_msg()   
        car_pose.pose.position.x = self.transform_callback().transform.translation.x
        car_pose.pose.position.y = self.transform_callback().transform.translation.y
        car_pose.pose.position.z = self.transform_callback().transform.translation.z
        car_pose.pose.orientation.x = self.transform_callback().transform.rotation.x
        car_pose.pose.orientation.y = self.transform_callback().transform.rotation.y
        car_pose.pose.orientation.z = self.transform_callback().transform.rotation.z
        car_pose.pose.orientation.w = self.transform_callback().transform.rotation.w
        self.pub_pose.publish(car_pose)
       
    def transform_callback(self):
        car_trans_Stamped = TransformStamped()
        try:
            now = rclpy.time.Time()
            car_trans_Stamped = self.buffer.lookup_transform(self.global_refFrame,self.car_refFrame,now)
            return car_trans_Stamped
        except TransformException as ex:
             self.get_logger().info(f'Could not transform : {ex}')

def main(args=None):
    rclpy.init(args=args)
    print("get_car_pose Initialized")
    get_car_pose_node = get_car_pose()
    rclpy.spin(get_car_pose_node)
    get_car_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()