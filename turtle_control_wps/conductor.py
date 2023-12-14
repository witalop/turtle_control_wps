import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool
import numpy as np
import json

class Conductor(Node):
    def __init__(self) -> None:
        super().__init__('conductor')

        #Parametros do nó
        self.sync_timer = 2
        self.queue_size = 10

        #goals
        self.goals =    [[-2, -0.5, 0], 
                        [-0.73, -0.57, 0], 
                        [-0.54, -1.53, 0], 
                        [0.18, -1.82, 0]]
        
        self.concluidos = 0


        #Posições
        self.x = None
        self.y = None
        self.yaw = None

        #Create subscribers
        self.pose_subscriber = self.create_subscription(Odometry,
                                                        "odom", 
                                                        self.pose_callback, 
                                                        self.queue_size)

        #Create publisher
        self.publisher_ = self.create_publisher(Pose2D, 'goal', 10)
        self.publisher_2 = self.create_publisher(Bool, 'goal_reached', 10)
        self.timer_publisher = self.create_timer(self.sync_timer, self.pub_callback)

    def pose_callback(self, msg):
        # print(msg)
        pose = msg.pose.pose

        self.x =  pose.position.x
        self.y =  pose.position.y
        _, _, self.yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        # self.get_logger().info(f'POSE x:{msg.x}    y:{msg.y}')

    def pub_callback(self):
        if self.x is not None and self.y is not None and self.yaw is not None:
            if len(self.goals)> self.concluidos:
                x_goal, y_goal, theta_goal = self.goals[self.concluidos]
                print(f'Goal: {x_goal}, {y_goal}, {theta_goal}')
                error_distance = self._calc_eroor(x_goal, y_goal, theta_goal)[0]
                print(f'Error distance: {error_distance}')
                if self._calc_eroor(x_goal, y_goal, theta_goal)[0] < 0.1:
                    self.concluidos += 1
                    print(f'Objetivo {self.concluidos}/{len(self.goals)} concluído')

                # print(type(x_goal), type(y_goal), type(theta_goal))
                msg = Pose2D()
                msg.x = float(x_goal)
                msg.y = float(y_goal)
                msg.theta = float(theta_goal)
                self.publisher_.publish(msg)

            else:
                print('Todos os objetivos concluídos')
                msg = Bool()
                msg.data = True
                self.publisher_2.publish(msg)

                #encerre o nó
                self.destroy_node()
                rclpy.shutdown()
                
    def _calc_eroor(self, x_goal, y_goal, theta_goal):
        error_x = x_goal - self.x
        error_y = y_goal - self.y
        distance = np.sqrt(error_x**2 + error_y**2)
        error_theta = theta_goal - self.yaw

        return distance, error_theta
    
def main():
    rclpy.init()
    node = Conductor()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()