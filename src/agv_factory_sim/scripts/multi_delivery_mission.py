#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import time

def create_pose(nav, x, y):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.w = 1.0
    return goal_pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Define coordinates based on your colored map
    BLUE_START = [-7.5, -7.5]
    GREY_PICKUP = [-7.5, 7.5]
    YELLOW_CNC = [[-5.0, 8.0], [0.0, 8.0], [5.0, 8.0]]

    # Mission Sequence
    for i, cnc in enumerate(YELLOW_CNC):
        # Go to Grey Pickup
        nav.goToPose(create_pose(nav, GREY_PICKUP[0], GREY_PICKUP[1]))
        while not nav.isTaskComplete(): pass
        print(f"Picked up cube {i+1} from Grey area.")

        # Go to Yellow CNC
        nav.goToPose(create_pose(nav, cnc[0], cnc[1]))
        while not nav.isTaskComplete(): pass
        print(f"Delivered to CNC {i+1}. Waiting 2s for process...")
        time.sleep(2)

    # Return Home
    nav.goToPose(create_pose(nav, BLUE_START[0], BLUE_START[1]))
    while not nav.isTaskComplete(): pass
    print("Mission complete. At Blue Start.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
