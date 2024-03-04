import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.get_logger().info('Funcionando...')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self._posicao3 = self.create_subscription(Odometry,"/alvo3/odometry" ,self.localizar, 10)
        self._posicao1 = self.create_subscription(Odometry,"/alvo1/odometry" ,self.callback1, 10)
        self._posicao4 = self.create_subscription(Odometry,"/alvo4/odometry" ,self.callback4, 10)
        self._posicao5 = self.create_subscription(Odometry,"/alvo5/odometry" ,self.callback5, 10)
        self._posicao2 = self.create_subscription(Odometry,"/alvo2/odometry" ,self.callback2, 10)
        #self.timer_ = self.create_timer(1.0, self.localizar)
        self.cmd_vel_msg = Twist()
        self.posicao1 = 0.0
        self.posicao4 = 0.0
        self.posicao5 = 0.0
        self.posicao2 = 0.0

    def callback1(self, msg):
        self.posicao1 = msg.pose.pose.position.x

    def callback4(self, msg):
        self.posicao4 = msg.pose.pose.position.x

    def callback5(self, msg):
        self.posicao5 = msg.pose.pose.position.x

    def callback2(self, msg):
        self.posicao2 = msg.pose.pose.position.x


    def localizar(self, msg):

        self.cmd_vel_msg.linear.x = 2.0  
        self.cmd_vel_msg.angular.z = 0.0
        
        x = msg.pose.pose.position.x
        if x>9:
            self.cmd_vel_msg.linear.x = 2.0
            self.cmd_vel_msg.angular.z = 0.5
            if self.posicao1<5:
                self.cmd_vel_msg.angular.z = -0.21
                if self.posicao4<-6:
                    self.cmd_vel_msg.angular.z = 0.31
                    if self.posicao5!=-2:
                        self.cmd_vel_msg.angular.z = -0.5
                        if self.posicao2!=6:
                            self.cmd_vel_msg.linear.x = 0.0  
                            self.cmd_vel_msg.angular.z = 0.0

                    


        
        print(f'O valor de x eh: {x}')
        
        self.publisher_.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()