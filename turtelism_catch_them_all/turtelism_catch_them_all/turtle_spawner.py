#!/usr/bin/env python3
from functools import partial
import random
import math
import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from turtlesim_catch_them_all_interfaces.msg import Turtle
from turtlesim_catch_them_all_interfaces.msg import TurtleArray
from turtlesim_catch_them_all_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):

    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawn_frequency", 1.0)
        
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.turtle_counter_ = 0
        self.alive_turtles_ = []
        self.spawn_server_client = self.create_spawn_server_client()
        self.kill_server_client = self.create_kill_server_client()
        self.alive_turtles_publisher = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.spawn_turtle_timer_ = self.create_timer(1.0 / self.spawn_frequency_, self.spawn_new_turtle)
        self.catch_turtle_service = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
    
    def create_kill_server_client(self):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service():
            self.get_logger().warn("Waiting for Kill service...")
        
        return client

    def create_spawn_server_client(self):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service():
            self.get_logger().warn("Waiting for Spawn service...")
        
        return client
    
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher.publish(msg)
        
    def spawn_new_turtle(self):
        request = self.create_spawn_request()
        future = self.call_server(self.spawn_server_client, request)
        future.add_done_callback(partial(self.callback_call_spawn, request=request))

    def callback_catch_turtle(self, catch_request, response):
        self.call_kill_server(catch_request.name)
        response.success = True
        return response
        
    def call_kill_server(self, turtle_name):
        request = Kill.Request()
        request.name = turtle_name
        future = self.call_server(self.kill_server_client, request)
        future.add_done_callback(partial(self.callback_call_kill, turtle_name=turtle_name))

    def create_spawn_request(self):
        self.turtle_counter_ += 1
        name = "turtle" + str(self.turtle_counter_)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*math.pi)
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        return request
      
    def call_server(self, client, request):
        return client.call_async(request)
        
    def callback_call_spawn(self, future, request):
        try:
            response = future.result()
            if response.name:
                self.get_logger().info("Turtle " + response.name + " is now alive")
                new_turtle = Turtle()
                new_turtle.name = request.name
                new_turtle.x = request.x
                new_turtle.y = request.y
                new_turtle.theta = request.theta
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
            
    def callback_call_kill(self, future, turtle_name):
        try:
            future.result()
            for (i, turtle) in enumerate(self.alive_turtles_):
                if turtle.name == turtle_name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
           self.get_logger().error("Service call failed %r" % (e,)) 
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()