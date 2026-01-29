import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def create_pose(nav, x, y, w=1.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = float(w)
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Wait for Nav2/A* Planner to be ready
    while not nav.isNav2Active():
        print("Waiting for Nav2...")
        time.sleep(1)

    # Set Initial Position (Pos 1)
    initial_pose = create_pose(nav, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # Define the Sequence of Waypoints (Positions 1-6)
    # Note: Coordinates are estimates; adjust based on your lab map view in RViz
    goal_poses = [
        create_pose(nav, 2.0, 0.0),   # Position 2 (Center)
        create_pose(nav, 2.5, 1.5),   # Position 3 (Grey Area - Left)
        create_pose(nav, 4.0, 1.5),   # Position 4 (Left)
        create_pose(nav, 2.0, 0.0),   # Back to Center (2)
        create_pose(nav, 2.5, -1.5),  # Position 5 (Right Area)
        create_pose(nav, 4.0, -1.5),  # Position 6 (Right)
        create_pose(nav, 0.0, 0.0)    # Back to Start (1)
    ]

    print("Starting A* Waypoint Sequence...")
    nav.followWaypoints(goal_poses)

    # Monitor Progress
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f'Visiting waypoint: {feedback.current_waypoint}')

    print("Mission Complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
