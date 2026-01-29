import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. Wait for Nav2 to be fully ready
    # This prevents the script from failing if the map is still loading
    while not nav.isNav2Active():
        print("Waiting for Navigation Stack to wake up...")
        time.sleep(1)

    # 2. Set the Initial Pose
    # This tells the A* planner exactly where the robot is starting in Gazebo
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    nav.setInitialPose(initial_pose)

    # 3. Define the Delivery Goal (e.g., Grey Room)
    # Update these X and Y coordinates to match your factory layout
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 4.5  
    goal_pose.pose.position.y = 1.5
    goal_pose.pose.orientation.w = 1.0

    print("Executing A* Path to Tool Delivery Point...")
    nav.goToPose(goal_pose)

    # 4. Monitor the Mission
    # This loop provides data for your Energy Efficiency study
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            # feedback.distance_remaining is key for SEC calculations
            print(f'Remaining distance: {feedback.distance_remaining:.2f} m')

    # 5. Check Final Result
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Mission Success: Tool Delivered to Grey Room!")
    elif result == TaskResult.CANCELED:
        print("Mission was canceled!")
    elif result == TaskResult.FAILED:
        print("Mission failed!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
