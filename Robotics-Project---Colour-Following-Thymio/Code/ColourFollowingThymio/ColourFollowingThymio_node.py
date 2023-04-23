import rclpy
from rclpy.node import Node
import tf_transformations
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import sys
import time
from copy import deepcopy
from math import sin, cos
from asebaros_msgs.msg import Event 

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        self.sensor_GroundDataL = int()
        self.sensor_GroundDataR = int()
        self.pose2 = (0, 0, 0)
        self.copy = (0, 0, 0)
        self.redcopy = (0, 0, 0)
        self.copyR = (0, 0, 0)
        self.copyleft = (0, 0, 0)
        self.outofboundscopy = (0, 0, 0)

        self.sensor_subscriber8 = self.create_subscription(Event, 'aseba/events/ground', self.sensor_callbackGround, 10)

        # Create a publisher for the topic 'cmd_vel'
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
    
    def sensor_callbackGround(self, msg8):
        self.sensor_GroundDataL = msg8.data[0]
        self.sensor_GroundDataR = msg8.data[1]

    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        self.pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return self.pose2
        
    def update_callback(self):

        x = self.pose2[0]
        y = self.pose2[1]
        z = self.pose2[2]

        xr = self.redcopy[0]
        yr = self.redcopy[1]
        zr = self.redcopy[2]

        ### Blue
        if self.sensor_GroundDataL > 800 and self.sensor_GroundDataR >800 and self.outofboundscopy[2]==0:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0
            self.copy = deepcopy(self.pose2)
            self.redcopy = (0, 0, 0)
            self.copyR = (0, 0, 0)
            self.copyleft = (0, 0, 0)
            self.vel_publisher.publish(cmd_vel)

        xc = self.copy[0]
        yc = self.copy[1]
        zc = self.copy[2]
        
        # Turning Condition (if the robot is 0.2 meters from the copy, it turns)
        if (((y-self.copy[1]) > 0.2) or ((x-self.copy[0]) > 0.2) or ((y-self.copy[1])<-0.2) or ((x-self.copy[0])<-0.2)) and (cos(z-zc))>=0.0 and self.copyR[2]==0 and self.copyleft[2]==0:
           cmd_vel = Twist() 
           cmd_vel.angular.z = 0.3
           self.redcopy = deepcopy(self.pose2)
           self.vel_publisher.publish(cmd_vel)

        # Stop turning --> moving condition (if the cosine of the difference between the yaw of the copy and 
        # the yaw of the robot is greater than 0, it stops turning and it moves forward)
        if cos(z-zc)<-0.30 and (abs(self.sensor_GroundDataR-self.sensor_GroundDataL) <2) and (self.copyleft[2] ==0 or self.copyR[2] ==0):
           cmd_vel = Twist()
           cmd_vel.linear.x = 0.01 
           cmd_vel.angular.z = 0.0
           self.vel_publisher.publish(cmd_vel)

        # Deepcopies for the adjustment
        if self.sensor_GroundDataR == 862 and self.sensor_GroundDataL == 730:
            self.copyR = deepcopy(self.pose2)

        if self.sensor_GroundDataR == 730 and self.sensor_GroundDataL == 862:
            self.copyleft = deepcopy(self.pose2)
        
        # Adjustments 
        if self.copyR[1] !=0 and abs(z-self.copyR[2]) <0.1:
           cmd_vel = Twist() 
           cmd_vel.angular.z = -0.1
           cmd_vel.linear.x = 0.0
           self.vel_publisher.publish(cmd_vel)

        if self.copyR[1] !=0 and abs(z-self.copyR[2]) >0.1:
           cmd_vel = Twist() 
           cmd_vel.angular.z = 0.0
           cmd_vel.linear.x = 0.1
           self.vel_publisher.publish(cmd_vel)

        if self.copyleft[2] !=0 and abs(z-self.copyleft[2]) <0.1:
           cmd_vel = Twist() 
           cmd_vel.angular.z = 0.1
           cmd_vel.linear.x = 0.0
           self.vel_publisher.publish(cmd_vel)

        if self.copyleft[2] !=0 and abs(z-self.copyleft[2]) >0.1:
           cmd_vel = Twist() 
           cmd_vel.angular.z = 0.0
           cmd_vel.linear.x = 0.1
           self.vel_publisher.publish(cmd_vel)

        # White adjustments (out of bound)
        if self.sensor_GroundDataR == 1023 and self.sensor_GroundDataL == 1023:
           cmd_vel = Twist() 
           self.outofboundscopy = deepcopy(self.pose2)
        
        if self.outofboundscopy[2] !=0 and (abs(z-self.outofboundscopy[2])<0.4):
           cmd_vel = Twist() 
           cmd_vel.angular.z = 0.1
           cmd_vel.linear.x = 0.0
           self.vel_publisher.publish(cmd_vel)
        
        if self.outofboundscopy[2] !=0 and (abs(z-self.outofboundscopy[2])>0.4):
           cmd_vel = Twist() 
           cmd_vel.angular.z = 0.0
           cmd_vel.linear.x = 0.1
           self.outofboundscopy = (0, 0, 0)
           self.vel_publisher.publish(cmd_vel)
        
        # Green out of bound adjustment (turning right if it sees green)
        if self.sensor_GroundDataL ==672 or self.sensor_GroundDataR==672:
           cmd_vel = Twist() 
           cmd_vel.angular.z = -0.3
           cmd_vel.linear.x = 0.0
           self.vel_publisher.publish(cmd_vel)

        print(self.sensor_GroundDataL, self.sensor_GroundDataR)

def main():

    rclpy.init(args=sys.argv)
    
    node = ControllerNode()
    node.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.stop()


if __name__ == '_main_':
    main()
