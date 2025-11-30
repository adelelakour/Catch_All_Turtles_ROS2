#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

import math
import random
from functools import partial


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.get_logger().info("Node has been started.")

        # Create client for spawn service
        self.turtle_spawner_client_ = self.create_client(Spawn, "/spawn")

        self.prefix_name = "turtle"
        self.turtle_counter_ = 1

        # Spawn a new turtle every 2 seconds
        self.timer = self.create_timer(2.0, self.spawn_new_turtle)

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
