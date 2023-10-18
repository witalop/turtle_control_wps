import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose

#TODO MONITORAR APENAS O ERRO, ACHO QUE QUANDO FICA MT PEQUENO ELE TA CRASHANDO. PODE DIMINUIR A FREQUENCIA DE MSG TBM

class TurtleControl(Node):
    def __init__(self) -> None:
        super().__init__('turtle_control_wps')

        #Constantes do robo
        self.v_max = 0.2
        self.kw = 3

        #Parametros do nó
        self.sync_timer = 0.5
        self.queue_size = 10

        #Inicializações
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    def init_publisher(self):
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', self.queue_size)
        self.timer_publisher = self.create_timer(self.sync_timer, self.pub_callback)
        print('Publisher inicializado')

    def init_subscribers(self):
        self.subscription_pose = self.create_subscription(Pose, 
                                                          'turtle1/pose', 
                                                          self.pose_callback,
                                                          self.queue_size)


        self.subscription_goal = self.create_subscription(Pose2D, 
                                                          'goal', 
                                                          self.goal_callback,
                                                          self.queue_size)
        print('Subscribers inicializados')

    def init_variables(self):
        self.x_goal = None
        self.y_goal = None
        self.theta_goal = None

        self.x = None
        self.y = None
        self.theta = None
        print('Variáveis inicializadas')

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        # self.get_logger().info(f'POSE x:{msg.x}    y:{msg.y}')

    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta
        # self.get_logger().info(f'GOAL x:{msg.x}    y:{msg.y}')


    def pub_callback(self):
        msg = Twist()

        if any(item is None for item in [self.x, self.y, self.theta, 
                                         self.x_goal, self.y_goal, self.theta_goal]):
            self.publisher.publish(msg)

        else:
            #Computar o erro
            x_error = self.x_goal - self.x
            y_error = self.y_goal - self.y
            # print(f'ERROS x: {x_error}    y: {y_error}')

            p = (x_error**2 + y_error**2)**0.5
            a = math.atan(y_error/x_error) - self.theta
            print(f'ERROS p: {p}    a: {a}')

            #Implementar o controle
            v = self.v_max * math.tanh(p)
            w = self.kw * a
            
            #Publicar msg
            if not (abs(p)<=0.1 and abs(a)<=0.1):
                msg.linear.x = v
                msg.angular.z = w

            self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    node = TurtleControl()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()