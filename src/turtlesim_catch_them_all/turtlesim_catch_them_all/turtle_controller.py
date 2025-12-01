#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

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

        #creaate a service client to catch turtles
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")

        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)


    def callback_alive_turtles_list(self, msg:TurtleArray ):
        
        self.turtle_to_catch = msg.turtles[0] if len(msg.turtles) > 0 else None

        if len(msg.turtles) > 0:
            self.turtle_to_catch = msg.turtles[0]
        else:
            self.turtle_to_catch = None

    def call_catch_turtle_service(self, turtle_name):
        self.catch_turtle_client_.wait_for_service(1.0)
        request = CatchTurtle.Request()
        request.name = turtle_name
        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(self.callback_call_catch_turtle_service)

    def callback_call_catch_turtle_service(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Turtle caught successfully.")
            else:
                self.get_logger().info("Failed to catch the turtle.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    

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
            self.call_catch_turtle_service(self.turtle_to_catch.name)
            self.turtle_to_catch = None   #reset the turtle to catch

        self.cmd_lev_pub_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
