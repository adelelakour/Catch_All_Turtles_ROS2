#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        
        #members
        self.pose_ : Pose = None
        self.turtle_to_catch:Turtle = None

        
        #subscriber to turtle1/pose to receive the turtle's pose
        self.pose_sub_ = self.create_subscription(Pose, "turtle1/pose", self.callback_pose_sub, 10)

        #publisher to /turtle1/cmd_vel to control the turtle
        self.cmd_lev_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        #subscriber to /alive_turtles to receive a list of alive turtles
        self.alive_turtles_list_sub = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles_list, 10)

        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)



    def callback_alive_turtles_list(self, msg:TurtleArray ):
        
        self.turtle_to_catch = msg.turtles[0] if len(msg.turtles) > 0 else None

        if len(msg.turtles) > 0:
            self.turtle_to_catch = msg.turtles[0]
        else:
            self.turtle_to_catch = None



    def callback_pose_sub(self, turtle_pose: Pose):
        self.pose_ = turtle_pose


    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch == None:
            return
        
        destination_x = self.turtle_to_catch.x - self.pose_.x
        destination_y = self.turtle_to_catch.y - self.pose_.y
        distance = math.sqrt(destination_x * destination_x + destination_y * destination_y)

        cmd = Twist()

        if distance > 0.5:
            cmd.linear.x = 2 * distance       #linear velocity is proportional to the distance. Kp can be tuned

            goal_theta = math.atan2(destination_y, destination_x)
            delta_theta = goal_theta - self.pose_.theta
            if delta_theta > math.pi:
                delta_theta -= 2*math.pi
            elif delta_theta < -math.pi:
                delta_theta += 2*math.pi
            
            cmd.angular.z = 6 * delta_theta

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_lev_pub_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
