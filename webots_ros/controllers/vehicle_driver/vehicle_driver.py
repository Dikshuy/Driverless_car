# Copyright 1996-2019 Cyberbotics Ltd.
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

#!/usr/bin/env python
"""vehicle_driver controller."""
import rospy
import math
from nav_msgs.msg import Odometry
from vehicle import Driver
#from cars_stage1.msg import ControlPoints, VehicleInfo, VehicleInfoArray,Location,CarDimensions,CarVelocity
from geometry_msgs.msg import Point, Pose, Quaternion,  Twist, Vector3

rospy.init_node('vehicle_driver', anonymous=True)
pub = rospy.Publisher('/ego_vehicle', Odometry , queue_size=20)
rate = rospy.Rate(10)
odom = Odometry()

driver = Driver()
driver.setSteeringAngle(0)
driver.setCruisingSpeed(20)
gps = driver.getGPS("gps")
gps.enable(1)

while driver.step() != -1:
    driver.setSteeringAngle(0)
    print(gps.getValues()[0],gps.getValues()[2])
    x= gps.getValues()[0]
    y = gps.getValues()[2]
    odom.pose.pose = Pose(Point(x,y,0),Quaternion(0,0,0,0))
    print(gps.getSpeed())
    vel = gps.getSpeed()
    odom.twist.twist = Twist(Vector3(vel,0,0),Vector3(0,0,0))
    pub.publish(odom)
