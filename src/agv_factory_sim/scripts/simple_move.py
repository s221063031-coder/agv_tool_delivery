import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AGVNavigator(Node):
    def __init__(self):
        super().__init__('agv_navigator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def move(self, linear_speed, duration):
        msg = Twist()
        msg.linear.x = float(linear_speed)
        self.execute_cmd(msg, duration)

    def turn(self, angular_speed, duration):
        msg = Twist()
        msg.angular.z = float(angular_speed)
        self.execute_cmd(msg, duration)

    def execute_cmd(self, msg, duration):
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        # Stop after each segment
        self.publisher_.publish(Twist()) 
        time.sleep(1.0) 

def main():
    rclpy.init()
    agv = AGVNavigator()

    # --- START OF SEQUENCE TO GREY ROOM ---
    print("Mission Started: Moving to Grey Room")
    agv.move(0.4, 5.0)  # Move forward for 5 seconds
    agv.turn(0.5, 3.0)  # Turn left (approx 90 degrees)
    agv.move(0.4, 4.0)  # Enter the room
    print("Mission Complete: Arrived at Grey Room")
    # --------------------------------------

    agv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
