#!/usr/bin/env python3
from functools import partial
import rclpy
import math
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_catch_them_all_interfaces.msg import Turtle
from turtlesim_catch_them_all_interfaces.msg import TurtleArray
from turtlesim_catch_them_all_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle_to_catch_ = None
        
        self.pose_ = None
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turtle_subscriber_ = self.create_subscription(TurtleArray, 'alive_turtles', self.callback_alive_turtles, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        self.catch_turtle_server_client = self.create_catch_turtle_server_client()
        
    def callback_turtle_pose(self, msg):
        self.pose_ = msg
        
    def callback_alive_turtles(self, msg):
        if not msg.turtles:
            return
        
        closest_turtle = None
        closest_turtle_distance = None
            
        for turtle in msg.turtles:
            dist_x = turtle.x - self.pose_.x
            dist_y = turtle.y - self.pose_.y
            distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)
            
            if not closest_turtle or distance < closest_turtle_distance:
                closest_turtle = turtle
                closest_turtle_distance = distance

        self.turtle_to_catch_ = closest_turtle
            
    def create_catch_turtle_server_client(self):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service():
            self.get_logger().warn("Waiting for Spawn service...")
        
        return client

    def call_catch_turtle_server(self, turtle_name):
        request = CatchTurtle.Request()
        request.name = turtle_name
        future = self.call_server(self.catch_turtle_server_client, request)
        future.add_done_callback(partial(self.callback_call_catch, turtle_name=turtle_name))
        
    def call_server(self, client, request):
        return client.call_async(request)
        
    def callback_call_catch(self, future, turtle_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Turtle " + turtle_name + " catched!")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    def control_loop(self):
        if not self.pose_ or not self.turtle_to_catch_:
            return
        
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        
        msg = Twist()
        
        if distance > 0.5:
            msg.linear.x = 2*distance
            
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
                
            msg.angular.z = 6*diff
        else:
            msg.linear.x = 0.0 
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None
            
        
        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()