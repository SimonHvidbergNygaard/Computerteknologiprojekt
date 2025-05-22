#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Ryan Shim, Gilbert
import time 
import rclpy

from turtlebot3_example.turtlebot3_obstacle_detection.turtlebot3_obstacle_detection \
    import Turtlebot3ObstacleDetection



def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()

    # turtlebot3_obstacle_detection.read_light_sensor()

    start_time = time.time()
    end_time = start_time + 60.0

    while time.time() < end_time:
        rclpy.spin_once(turtlebot3_obstacle_detection, timeout_sec=0.1)

    turtlebot3_obstacle_detection.robot_stop()

    print("\nCollision count: ", turtlebot3_obstacle_detection.collision_counter)
    print("\n Speed_updates: ", turtlebot3_obstacle_detection.speed_updates)
    print("\n Speed_accumulation: ", turtlebot3_obstacle_detection.speed_accumulation)
    print("\n average linear speed: ", turtlebot3_obstacle_detection.average_linear_speed())
    print("\n Victims: ", turtlebot3_obstacle_detection.victim_counter)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()
