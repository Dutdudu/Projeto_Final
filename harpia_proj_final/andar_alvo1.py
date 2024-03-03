import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.get_logger().info('Funcionando...')
        self._posicao1 = self.create_subscription(Odometry,"/alvo3/odometry" ,self.localizar, 10)
        #self.timer_ = self.create_timer(1.0, self.localizar)
        #self.cmd_vel_msg = Twist()

    def localizar(self, msg):

        x = msg.pose.pose.position.x
        print(f'O valor de x eh: {x}')
        
        
        #self.cmd_vel_msg.linear.x = 0.5  
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