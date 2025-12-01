#!/usr/bin/env python3
import rclpy
import math
import random
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.srv import Kill

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.get_logger().info("Node has been started.")

        #class members
        self.alive_turtles = []
        self.turtle_counter_ = 1
        self.prefix_name = "turtle"
        

        #create a publisher to send the list of alive turtles 
        self.alive_turtles_pub = self.create_publisher(TurtleArray, "alive_turtles", 10)

        # Create client for spawn service
        self.turtle_spawner_client_ = self.create_client(Spawn, "/spawn")
        
        # Spawn a new turtle every 2 seconds
        self.timer = self.create_timer(2.0, self.spawn_new_turtle)

        #create a service to catch turtles
        self.catch_turtle_srv_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle_srv)

        #create a client to /Kill service
        self.turtle_killer_client_ = self.create_client(Kill, "/kill")



    def callback_catch_turtle_srv(self, request:CatchTurtle.Request, response:CatchTurtle.Response):
        #call the kill service to remove the turtle
        self.call_kill_service(request.name) 
        response.success = True
        return response


    def call_kill_service(self, turtle_name):
        self.turtle_killer_client_.wait_for_service(1.0)
        request = Kill.Request()
        request.name = turtle_name
        future = self.turtle_killer_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, turtle_name))
    
    def callback_call_kill_service(self, turtle_name, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle {turtle_name} killed.")
            #remove the turtle from alive_turtles list
            self.alive_turtles = [turtle for turtle in self.alive_turtles if turtle.name != turtle_name]
            self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Kill service failed: {e}")


    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.alive_turtles_pub.publish(msg)


    def spawn_new_turtle(self):
        self.turtle_counter_ += 1
        name = self.prefix_name + str(self.turtle_counter_)

        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi)

        self.get_logger().info(f"Spawning turtle: {name}")
        self.call_spawn_service(x, y, theta, name)

    def call_spawn_service(self, x, y, theta, name):
        # Wait for service to be available
        while not self.turtle_spawner_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting: /spawn service is not available...")

        # Prepare request
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        # Call service asynchronously
        future = self.turtle_spawner_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, request))

    def callback_call_spawn_service(self, request, future):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"Turtle spawned: {response.name}")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = request.x
                new_turtle.y = request.y
                new_turtle.theta = request.theta
                self.alive_turtles.append(new_turtle)
                self.publish_alive_turtles()

        except Exception as e:
            self.get_logger().error(f"Spawn service failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
