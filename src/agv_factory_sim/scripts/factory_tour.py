import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class FactoryMission(Node):
    def __init__(self):
        super().__init__('factory_mission')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def cmd(self, linear, angular, duration):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        
        start = time.time()
        while (time.time() - start) < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        self.publisher.publish(Twist()) # Stop between steps
        time.sleep(1.0)

def main():
    rclpy.init()
    agv = FactoryMission()

    print("Step 1: Position 1 to Center 2")
    agv.cmd(0.4, 0.0, 4.0) 

    print("Step 2: Center 2 to Grey Area 3")
    agv.cmd(0.0, 1.57, 1.0) # Turn 90 deg Left
    agv.cmd(0.4, 0.0, 3.0) 

    print("Step 3: Reverse from 3 to 4")
    agv.cmd(-0.4, 0.0, 3.0) # Back to center
    agv.cmd(0.0, 3.14, 1.0) # Turn 180 deg
    agv.cmd(0.4, 0.0, 5.0)  # Toward Pos 4

    print("Step 4: Pos 4 to Center 2 to Pos 5")
    agv.cmd(-0.4, 0.0, 5.0) # Back to center
    agv.cmd(0.0, 1.57, 1.0) # Turn 90 deg Right
    agv.cmd(0.4, 0.0, 4.0)  # Toward Pos 5

    print("Step 5: Pos 5 to Center 2 to Pos 6")
    agv.cmd(-0.4, 0.0, 4.0) # Back to center
    agv.cmd(0.0, 3.14, 1.0) # Turn 180 deg
    agv.cmd(0.4, 0.0, 4.0)  # Toward Pos 6

    print("Step 6: Returning to Start 1")
    agv.cmd(-0.4, 0.0, 4.0) # Back to center
    agv.cmd(0.0, 1.57, 1.0) # Turn toward Pos 1
    agv.cmd(0.4, 0.0, 4.0) 

    print("Mission Success!")
    agv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
