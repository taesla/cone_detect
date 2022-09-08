#! /usr/bin/env python

import rospy
import numpy as np
import math
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistWithCovarianceStamped
import message_filters

map_x = []
map_y = []
map_yaw = []
# paramters
dt = 0.1

# k =0.5
k = 1  # control gain

# ERP42 PARAMETERS
LENGTH = 1.600
WIDTH = 1.160

# # GV70 PARAMETERS
# LENGTH = 4.715
# WIDTH = 1.910
max_steering = np.radians(30) # max_angle = 30 degree
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians

def normalize_angle(angle):
	while angle > np.pi:
		angle -= 2.0 * np.pi

	while angle < -np.pi:
		angle += 2.0 * np.pi

	return angle


def stanley_control(x, y, yaw, v, map_xs, map_ys, map_yaws, L):
	# find nearest point
	global k
	min_dist = 1e9
	min_index = 0
	n_points = len(map_xs)
	front_x = x + L * np.cos(yaw)
	front_y = y + L * np.sin(yaw)
	for i in range(n_points):
		dx = front_x - map_xs[i]
		dy = front_y - map_ys[i]

		dist = np.sqrt(dx * dx + dy * dy)
		if dist < min_dist:
			min_dist = dist
			min_index = i
                        min_index = min_index
	# compute cte at front axle
	map_x = map_xs[min_index]
	map_y = map_ys[min_index]
	map_yaw = map_yaws[min_index]
	dx = map_x - front_x
	dy = map_y - front_y



	# control law
#	yaw_term = normalize_angle(map_yaw - yaw) * np.sin(np.pi/2 / (1+v/5))
	yaw_term = normalize_angle(map_yaw - yaw) #heading error
	perp_vec = [np.cos(map_yaw + np.pi/2), np.sin(map_yaw + np.pi/2)]
	cte = np.dot([dx, dy], perp_vec) # Cross track error
	print(cte,yaw_term)
	cte_term = np.arctan2(k*cte, v) # cross track error
	w_yaw = 1
	w_cte = 1
	# steering
	steer = w_yaw * yaw_term + w_cte * cte_term
	steer = np.clip(steer, -max_steering, max_steering) # limit the steering angle
	return steer#, [w_yaw, w_cte, k, yaw_term, cte_term]

def callback1(msg):

        global vel

        vel = np.sqrt(msg.twist.twist.linear.x **2 + msg.twist.twist.linear.y**2)

def callback(msg):
	global map_x,map_y,map_yaw

	for i in range(len(msg.poses)-1):
		map_x.append(msg.poses[i].pose.position.x)
		map_y.append(msg.poses[i].pose.position.y)
		map_yaw.append(euler_from_quaternion(msg.poses[i].pose.orientation.x,msg.poses[i].pose.orientation.y,msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w))

def callback2(msg):
	yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
	# print(yaw)
        #vel = np.sqrt(velocity.twist.twist.linear.x **2 + velocity.twist.twist.linear.y **2)
	steer = stanley_control(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, vel, map_x, map_y, map_yaw, 1.06)
        #print(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw,map_x[3],map_y[3].map_yaw[3])
        #print(map_y)
	print(steer)
	steer_pub.publish(steer)

if __name__ == '__main__':
    rospy.init_node('stanley_control')
    rospy.Subscriber('/fastlap_path',Path,callback)

    odom_sub = rospy.Subscriber('/odom',Odometry,callback2)
    vel_sub = rospy.Subscriber('/ublox_gps/fix_velocity',TwistWithCovarianceStamped,callback1)

    steer_pub = rospy.Publisher('/steer',Float32,queue_size=1)

    rospy.spin()
