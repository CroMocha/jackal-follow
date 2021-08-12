import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from geometry_msgs.msg import Twist

prev_error = 0
integral_e = 0
integral_cap = 10 # Prevent integral term from getting too large
offset_distance = 1.0 # The following jackal should be a set distance away from the front jackal
min_scanner_range = 0.5 # Remove invalid lidar scanner readings
max_scanner_range = 10.0
max_vel = 3.0
angular_multiplier = 4

rospy.init_node("follow")

publisher = rospy.Publisher('/jackal0/jackal_velocity_controller/cmd_vel', Twist, queue_size = 1)

def rad_to_degree(rad):
    # radians to degree conversion
    degree = (rad/np.pi)*180
    return degree

def calculate_radius_of_curvature(angle, distance):
    # Calculate radius_of_curvature using the angle and distance of centroid
    arc_radius = (distance/2.0)/np.sin(angle)
    arc_length = arc_radius * 2 * angle

    # prevent arc_radius from going to infinity when angle is small ( -> 0)
    if arc_radius > 1000000:
        arc_radius = 1000000
    return arc_radius, arc_length

def calculate_angular_vel(linear_vel,radius):
    # Calculate angular velocity using the radius of curvature and linear velocity
    angular_vel = linear_vel/radius
    return angular_multiplier*angular_vel

def pid(distance):
    # PID controller for linear velocity
    global prev_error 
    global integral_e 

    kp = 3.0
    ki = 0.0
    kd = 0.5

    error = distance - offset_distance

    # proportional term
    proportional_e = error
    # derivative term
    derivative_e = error - prev_error
    # integral term
    integral_e += error
    if integral_e >= integral_cap:
        integral_e = integral_cap
    if integral_e <= -integral_cap:
        integral_e = -integral_cap

    prev_error = error

    pid_linear_vel = kp*proportional_e + kd*derivative_e + ki*integral_e
    return pid_linear_vel

def scan_cb(msg):
    # Get lidar scan from jackal and compute kinematics
    rangelist = []
    angledegreelist = []
    angleradianlist = []
    curr_angle = msg.angle_max
    for object_range in msg.ranges:
        # remove invalid lidar readings
        if object_range != float("inf") and object_range > min_scanner_range and object_range < max_scanner_range:
            rangelist.append(object_range)
            angleradianlist.append((curr_angle))
            angledegreelist.append(rad_to_degree(curr_angle))
        curr_angle -= msg.angle_increment
    centrepoint = len(rangelist)//2
    if len(rangelist) != 0:
        # find centroid of lidar points
        print("distance")
        print(rangelist[centrepoint])
        print("angle")
        print(angleradianlist[centrepoint])

        distance = rangelist[centrepoint]
        angle = angleradianlist[centrepoint]

        arc_radius, arc_length = calculate_radius_of_curvature(angle,distance)
        
        pid_linear_vel = pid(distance)

        if pid_linear_vel > max_vel:
            pid_linear_vel = max_vel
        if pid_linear_vel < -max_vel:
            pid_linear_vel = -max_vel

        angular_vel = calculate_angular_vel(pid_linear_vel,arc_radius)

        # if angular_vel > max_vel:
        #     angular_vel = max_vel
        # if angular_vel < -max_vel:
        #     angular_vel = -max_vel

        print("linear vel:")
        print(pid_linear_vel)
        print("angular")
        print(angular_vel)

        twist = Twist()
        twist.linear.x = pid_linear_vel
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_vel
        publisher.publish(twist)

rospy.Subscriber("/jackal0/front/scan", LaserScan, scan_cb, queue_size=1)

rospy.spin()