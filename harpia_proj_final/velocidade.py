import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.get_logger().info('Funcionando...')
        self.publisher_ = self.create_subscription(Odometry, '/alvo5/odometry',self.publish_command, 10)
        #self.timer_ = self.create_timer(1.0, self.publish_command)
        self.cmd_vel_msg = Twist()

    def publish_command(self, msg):
        x = msg.pose.pose.position.x
        print(f'A posicao do alvo 5 eh: {x}')
        
        #self.cmd_vel_msg.linear.x = 5.0  
        #self.cmd_vel_msg.angular.z = 0.0  
        #self.publisher_.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()