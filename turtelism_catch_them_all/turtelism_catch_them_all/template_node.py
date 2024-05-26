#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class CustomNode(Node): # TODO: change name

    def __init__(self):
      super().__init__("node_name") # TODO: change name
 
def main(args=None):
    rclpy.init(args=args)
    node = CustomNode() # TODO: change name
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()