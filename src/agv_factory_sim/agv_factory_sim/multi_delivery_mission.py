import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def create_pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Define Coordinates based on your Gazebo layout
    BLUE_START = [-7.0, -7.0]
    GREY_PICKUP = [-7.0, 7.0]
    YELLOW_CNC = [
        [-5.0, 8.5],  # 1st CNC (Left)
        [0.0, 8.5],   # 2nd CNC (Middle)
        [5.0, 8.5]    # 3rd CNC (Right)
    ]

    print("Starting Mission: Moving to Grey Area...")
    
    for i, cnc_pos in enumerate(YELLOW_CNC):
        # 1. Move to Grey Area to pick up cube
        nav.goToPose(create_pose(GREY_PICKUP[0], GREY_PICKUP[1]))
        while not nav.isTaskComplete(): pass
        print(f"Cube {i+1} picked up from Grey Area.")

        # 2. Move to specific Yellow CNC point
        nav.goToPose(create_pose(cnc_pos[0], cnc_pos[1]))
        while not nav.isTaskComplete(): pass
        print(f"Cube {i+1} delivered to Yellow CNC {i+1}.")

    # 3. Return to Blue Start Point
    print("All deliveries complete. Returning to Blue Start Area...")
    nav.goToPose(create_pose(BLUE_START[0], BLUE_START[1]))
    while not nav.isTaskComplete(): pass

    print("Mission Finished Successfully.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
