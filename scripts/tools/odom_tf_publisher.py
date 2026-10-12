import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/mobile_base_controller/odom',
            self.odom_callback,
            10)
        self.get_logger().info("OdomTfPublisher AVVIATO! In attesa di odometria...")

    def odom_callback(self, msg):
        # Messaggio di debug (stampa solo ogni 2 secondi per non intasare il terminale)
        self.get_logger().info("Sto ricevendo l'odometria e pubblicando il TF!", throttle_duration_sec=2.0)
        
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.br.sendTransform(t)

def main(args=None):
    # FIX: Passiamo sys.argv così assorbe --ros-args dal launch file!
    rclpy.init(args=sys.argv)
    node = OdomTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()