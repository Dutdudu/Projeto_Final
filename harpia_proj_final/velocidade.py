import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.publish_command)
        self.cmd_vel_msg = Twist()

    def publish_command(self):
        
        self.cmd_vel_msg.linear.x = 5.0  
        self.cmd_vel_msg.angular.z = 0.0  
        self.publisher_.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()