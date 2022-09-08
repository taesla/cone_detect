#!/usr/bin/env python
from re import X
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion
from math import atan2


x_lst = []
y_lst = []
range_lst=[]

centroid_x = 0.0
centroid_y = 0.0

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print("[roll]:",roll,"[pitch]:",pitch,"[yaw]:",yaw)

def callback(pose_array):

    marker_ests = MarkerArray()
    marker_ests.markers = []
    
    marker_est = Marker()
    marker_est.header.frame_id = "/velodyne"
    marker_est.ns = "marker"
    #marker_est.id = 42+i
    marker_est.type = Marker.CUBE
    marker_est.action = Marker.ADD
    
    
    
    for i in range(0,len(pose_array.poses)):
        cone_x = pose_array.poses[i].position.x
        cone_y = pose_array.poses[i].position.y
        cone_circle = pow(cone_x,2) + pow(cone_y,2)
        if (np.sqrt(cone_circle) <= 3):
            
            cnt_point = len(pose_array.poses)
            pub_msgs = Twist()
            print("cone is closely detected!")
            x_lst.append(pose_array.poses[i].position.x)
            y_lst.append(pose_array.poses[i].position.y)
            centroid_x = sum(x_lst) / float(cnt_point)
            centroid_y = sum(y_lst) / float(cnt_point)
            marker_est.pose.position.x = centroid_x
            marker_est.pose.position.y = centroid_y
            marker_est.pose.position.z = 0
            marker_est.pose.orientation.w = 1
            marker_est.color.r, marker_est.color.g, marker_est.color.b = (0, 255, 0)
            marker_est.color.a = 0.5
            marker_est.scale.x, marker_est.scale.y, marker_est.scale.z = (0.06, 0.06, 0.06)
            marker_ests.markers.append(marker_est)
            center_markers.publish(marker_ests)
        

            goal_x = centroid_x - roll
            goal_y = centroid_y - pitch

            angle_to_goal = atan2(goal_y, goal_x)

            if abs(angle_to_goal - yaw) > 0.1:
                pub_msgs.linear.x = 0.0
                pub_msgs.angular.z = angle_to_goal
            else:
                pub_msgs.linear.x = 0.5
                pub_msgs.angular.z = 0.0

            center_line.publish(pub_msgs)  


            

        



    
        
if __name__=='__main__':
    # Init ROS
    rospy.init_node('centroid_line', anonymous=True)
    # Subscribers
    
    rospy.Subscriber('/adaptive_clustering/poses', PoseArray, callback)
    rospy.Subscriber('/imu/data',Imu,get_rotation)
    # Publishers
    center_line = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    center_markers = rospy.Publisher('/center_point', MarkerArray, queue_size=1)
    # Spin
    rospy.spin()
   