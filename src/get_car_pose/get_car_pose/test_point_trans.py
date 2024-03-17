import rclpy
import tf2_ros
from rclpy.node import Node
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException

class test_point_trans(Node):
    def __init__(self):
        super().__init__("test_point_trans")

        self.pub_points_trans = self.create_publisher(PointStamped,"/test_point_trans",1)
        self.create_timer(1,self.timer_callback)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer,self)
    
    def timer_callback(self):

        point_source = PointStamped()
        t = self.get_clock().now()
        point_source.header.frame_id = "map"
        point_source.header.stamp = t.to_msg()
        point_source.point.x = 10.0
        point_source.point.y = 0.0
        point_source.point.z = 0.0
        point_target = self.buffer.transform(point_source,"base_link",rclpy.time.Duration(seconds=2))
        

        # point_target = PointStamped()
        # try:
        #     point_target = self.buffer.transform(point_source,"base_link",timeout=rclpy.time.Duration(seconds=1.0))
        #     self.pub_points_trans.publish(point_target)
        # except TransformException as ex:
        #      self.get_logger().info(f'Could not transform : {ex}')
        



def main(args=None):
    rclpy.init(args=args)
    print("test_point_transInitialized")
    test_point_trans_node = test_point_trans()
    rclpy.spin(test_point_trans_node)
    test_point_trans_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    