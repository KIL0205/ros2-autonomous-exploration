#!/usr/bin/env python3
"""
Super simple AMCL reinitializer
Run this after AMCL is started
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time

def main():
    rclpy.init()
    
    node = Node('blowup')

    node.get_logger().info("IM")
    
    reinit_client = node.create_client(Empty, '/reinitialize_global_localization') 
    nomotion_client = node.create_client(Empty, '/request_nomotion_update') 
    
    if not reinit_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Reinit service not available!")
        return
    
    if not nomotion_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn("Nomotion service not available, continuing anyway...")

    node.get_logger().info("GONNA")
    
    node.get_logger().info("Calling reinitialize service...")
    reinit_client.call_async(Empty.Request())
    
    node.get_logger().info("Done! Particles spread across map")
    
    node.get_logger().info("Sending no-motion updates...")
    
    for i in range(10000):
        nomotion_client.call_async(Empty.Request())
        time.sleep(0.15)
        
        if (i + 1) % 10 == 0:
            node.get_logger().info(f"Sent {i + 1} updates")

    node.get_logger().info("All done!")
    

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()