

#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import time 
import smbus
import RPi.GPIO as GPIO


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop angle: -90 ~ 90 deg')
        print('stop distance: 0.5 m')
        print('----------------------------------------------')
###################################### Variables ###############################################
        self.scan_ranges = []
        self.has_scan_received = False

        #Parameters for when to stop and default speed
        self.stop_distance_40cm = 0.40
        self.stop_distance_20cm = 0.20
        self.stop_distance_15cm = 0.16
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0
        # initializing the counters to be 0 
        self.speed_accumulation = 0.0
        self.speed_updates = 0.0
        self.collision_counter = 0.0
        self.victim_counter = 0.0
        # Creating read_light_sensor to read colors
        self.bus = smbus.SMBus(1)  # Only if not already initialized
        self.create_timer(2.0, self.read_light_sensor)
        self.bus.write_byte_data(0x44, 0x01, 0x05)

        #Setup for LED
        GPIO.setmode(GPIO.BCM)
        self.GPIO_LED = 17 #pin 17 on rpi
        GPIO.setup(self.GPIO_LED, GPIO.OUT)

        # Making a cooldown for collision counter
        self.last_collision_time = 0
        self.collision_cd = 3.0

         # Making a cooldown for victim counter
        self.last_victim_time = 0
        self.victim_cd = 1.0
###########################################################################################
################################### Pre-code ##############################################
        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg
        
    # Timer callback function, runs periodically
    def timer_callback(self):
        if self.has_scan_received:
            self.detect_obstacle()
            
###########################################################################################
################################### Robot stop function ##############################################
    def robot_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info(f'Robot stopping', throttle_duration_sec=2)

        self.cmd_vel_pub.publish(twist)

###########################################################################################
################################### Color and victim counter ##############################################

    def read_light_sensor(self):
        try:
            #Defining our colors - Reads 6 bytes of data, which returns the RGB values from a color sensor
            data = self.bus.read_i2c_block_data(0x44, 0x09, 6)
        
            RedColor=data[3] + data[2]/256
            GreenColor=data[1] + data[0]/256
            # Sensor Callibration
            BlueColor=(data[5] + data[4]/256)*1.55

        
           # Defining the dominating color - Further callibrations are the numbers listed below
            if RedColor > 0.76 * GreenColor and RedColor > BlueColor:
                self.get_logger().info(f"Red: {RedColor}")
            elif GreenColor > 0.5*RedColor and GreenColor * 0.76  > BlueColor:
                self.get_logger().info(f"Green: {GreenColor}")
            elif BlueColor > RedColor and BlueColor > 0.76 * GreenColor:
                 self.get_logger().info(f"Blue: {BlueColor}")
        
            self.get_logger().info(f"RGB{RedColor:.2f}, {GreenColor*0.76:.2f}, {BlueColor:.2f}")

            # Threshold for red to be spotting a victim, depending on lighting.
            threshold = 58.0
            # Initializing a cooldown for when it can read its victims
            current_time_victim = time.time()

            # Counter for "red" victims 
            if RedColor > threshold and RedColor > 0.76*GreenColor and RedColor > BlueColor:
                if current_time_victim - self.last_victim_time > self.victim_cd:
                    self.victim_counter += 1
                    self.last_victim_time = current_time_victim

                    # Makes the LED light up, when it spots a victim
                    self.get_logger().info('Victim found')
                    GPIO.output(self.GPIO_LED, True)
                    time.sleep(0.2)
                    GPIO.output(self.GPIO_LED, False)

        except Exception as e:
                self.get_logger().error(f"Failed to read light sensor: {e}")
       
###########################################################################################
################################### Speed-reading functions ##############################################

    # Calculates the average linear speed
    def average_linear_speed(self):
        if self.speed_updates == 0:
            return 0.0
        return self.speed_accumulation/self.speed_updates
    
    # updates the speed_accumulator with speed
    def get_speed_accumulation(self, speed):
        self.speed_accumulation += speed
    
    
    # Get incremented as well
    def get_speed_updates(self):
        self.speed_updates = self.speed_updates + 1
###########################################################################################
################################### Sensor cones for read ##############################################
    def detect_obstacle(self):
        total_range = len(self.scan_ranges)

        # Divide the scan into left, front, and right, farRight, farLeft
        left_start = int(total_range * (18) / 360)
        left_end = int(total_range * (54) / 360)
    
        front_start = int(total_range * (342) / 360)
        front_end = int(total_range * (18) / 360)
    
        right_start = int(total_range * (306) / 360)
        right_end = int(total_range * (342) / 360)

        farLeft_start = int(total_range * (54) / 360) 
        farLeft_end =  int(total_range * (90) / 360)

        farRight_start = int(total_range * (270) / 360)
        farRight_end = int(total_range * (306) / 360)

        # Get min distance from each section
        left_distance = min(self.scan_ranges[left_start:left_end])
        front_distance = min(self.scan_ranges[front_start:] + self.scan_ranges[:front_end])
        right_distance = min(self.scan_ranges[right_start:right_end])
        farLeft_distance = min(self.scan_ranges[farLeft_start:farLeft_end])
        farRight_distance = min(self.scan_ranges[farRight_start:farRight_end])
 
        twist = Twist()

###########################################################################################
################################### Movement functions ##############################################
        def trapped():
            self.get_logger().info('Robot is trapped! Executing escape maneuver...', throttle_duration_sec=2)
        
            # Rotate in place (turn around)
            if left_distance > right_distance:
                twist.linear.x = 0.0
                twist.angular.z = 1.5  # Turn left quickly
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.8)  # turning for 0.8 seconds approx 120degrees
            else:
                twist.linear.x = 0.0
                twist.angular.z = -1.5  # Turn right quickly
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.8)  # turning for 0.8 seconds approx 120 degrees
            


        def slowly_turn_right():
            twist.linear.x = 0.1
            twist.angular.z = -0.8
            self.get_logger().info(f'Obstacle on far left! Slowly turning! Front distance: {front_distance:.2f} m. Left distance: {left_distance:.2f} m', throttle_duration_sec=2)

        def sharp_turn_right():
            twist.linear.x = 0.12
            twist.angular.z = -1.5
            self.get_logger().info(f'Obstacle on front and left! Sharp turn! Front distance: {front_distance:.2f} m, Left distance: {left_distance:.2f} m', throttle_duration_sec=2)

        def slowly_turn_left():
            twist.linear.x = 0.1
            twist.angular.z = 0.8
            self.get_logger().info(f'Obstacle on far right! Slowly turning. Front distance: {front_distance:.2f} m, Right distance {right_distance:.2f} m', throttle_duration_sec=2)

        def sharp_turn_left():
            twist.linear.x = 0.12
            twist.angular.z = 1.5
            self.get_logger().info(f'Obstacle on front and right! Sharp turn. Front distance: {front_distance:.2f} m. Right distance: {right_distance:.2f} m', throttle_duration_sec=2)
            
###########################################################################################
################################### Obstacle-avoidance ##############################################
        # go forward!
        if front_distance < self.stop_distance_15cm:
            trapped()

        elif front_distance < self.stop_distance_40cm:

            # checking the sides! 
            # either left or right side is less than 20 cm
            if left_distance < self.stop_distance_20cm and right_distance < self.stop_distance_20cm:
                self.tele_twist

            # Turning to the right
            elif left_distance < right_distance: 
                # Turning to the right!
                if farLeft_distance < left_distance:
                    slowly_turn_right()
                else:
                    # Turning to the right
                    sharp_turn_right()

            elif right_distance < left_distance: # Turning to the left
                
                if farRight_distance < right_distance:
                    slowly_turn_left()
                else: 
                    # Turning to the left
                    sharp_turn_left()

            else:
                twist = self.tele_twist

        else:
            # No obstacle → continue with current command
            twist = self.tele_twist
        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)


###########################################################################################
################################### Collision counter ##############################################
        
        # Creating time space for collision counter
        current_time = time.time()
        
        #Collision counter if anything is within 16cm it counts as collision
        if (front_distance < self.stop_distance_15cm or
             right_distance < self.stop_distance_15cm or
             farRight_distance < self.stop_distance_15cm or
             left_distance < self.stop_distance_15cm or
             farLeft_distance < self.stop_distance_15cm):
                 #Creating cooldown
             if current_time - self.last_collision_time > self.collision_cd:
                self.collision_counter += 1 # Increments collisions
                self.last_collision_time = current_time
                self.get_logger().info(f'Total Collisions: {self.collision_counter}', throttle_duration_sec=3)

  #######################################################################################
        
        self.get_speed_updates() # Calling function, with how many speed updates we had
        self.get_speed_accumulation(twist.linear.x) # Calling function with the linear speed
