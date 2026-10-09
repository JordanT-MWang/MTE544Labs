# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        
        #added variable for functions#
        #starting velocity
        self.time = 0.1
        self.v = 0.25
        self.line_dist = 0
        self.line_dist_max = 5.0
        #starting angular velocity
        self.w = 0.5
        self.r_max = self.v/self.w
        self.radius_= self.r_max
        self.dr = 0.01
        self.r_min = 0.15
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        self.sprial_in=True
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qospub=QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist,'/cmd_vel', qospub)

        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        
        

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos
        )
        
        # ENCODER subscription

        self.create_subscription(
                    Odometry,
                    '/odom',
                    self.odom_callback,
                    qos
                )
        
        # LaserScan subscription 
        
        self.create_subscription(
                            LaserScan,
                            '/scan',
                            self.laser_callback,
                            qos
                        )
        
        self.create_timer(self.time, self.timer_callback)

    
        
    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        print("test4")
        values = [
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.angular_velocity.z,
            Time.from_msg(imu_msg.header.stamp).nanoseconds
        ]

        self.imu_logger.log_values(values)

        self.imu_initialized = True
        
    def odom_callback(self, odom_msg: Odometry):
        print("test3")
        #change the quaterion to euler
        quaternion = odom_msg.pose.pose.orientation
        th = euler_from_quaternion(quaternion)
        values = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            th,
            Time.from_msg(odom_msg.header.stamp).nanoseconds
        ]

        self.odom_logger.log_values(values)
        self.odom_initialized = True
                
    def laser_callback(self, laser_msg: LaserScan):
        print("test2")
        #used to seperate the values and add ; betwen the values
        ranges_str = ";".join(str(r) for r in laser_msg.ranges)
        values = [
            ranges_str,
            laser_msg.angle_increment,
            Time.from_msg(laser_msg.header.stamp).nanoseconds
        ]
        self.laser_logger.log_values(values)

        self.laser_initialized = True
                
    def timer_callback(self):
        print("test1")
        print
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        
        msg=Twist()

        #raduis will be v/w for these number be 0.4
        msg.linear.x = self.v
        msg.angular.z =self.w
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        #check if spiraling in. if so decrease radius of circle
        if self.sprial_in:
            #minus by preset sprial
            self.radius_ -=self.dr
            #check if radus is smaller then set min
            if self.radius_ <= self.r_min:
                #change direction for next loop
                self.sprial_in = False
        else:
            #outward sprial
            self.radius_ += self.dr
            #check if radius is bigger then circle 
            if self.radius_ >=self.r_max:
                #set so sprial in
                self.sprial_in = True

        #set velocity and calc the angular velocity
        msg.linear.x = self.v
        msg.angular.z = self.v/self.radius_
        
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        #add distance
        
        self.line_dist += self.v*self.time
        #check if its gone max distance
        

        msg.linear.x = self.v
        msg.angular.z = 0.0
        
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
    
        
