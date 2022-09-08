#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist , PoseStamped
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
import math

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def callback(msg):
    vel_msg = Twist()

    marker_array = MarkerArray()
    marker_array = msg


    # edges, nodes == "marker"
    loop_radius = marker_array.markers[3]
    edges = marker_array.markers[2]

    nodes = marker_array.markers[0]
    # print(len(nodes.points))
    # every x y z points of edges, nodes
    edges_points = edges.points
    nodes_points = nodes.points

    edges_size = len(edges_points)
    nodes_size = len(nodes_points)

    edges.text = str(edges_size)
    nodes.text = str(nodes_size)
    #print("size",len(edges.points))
    #print("edges_points",edges.points,"nodes_points",nodes.points)

    path_pub = PoseStamped()
    
    path_pub.header.frame_id = "map"

    #print("edges_size:",edges_size)

    #print("edges_points:",edges_points[0].x)
    

    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    for i in range(len(nodes.points)-1):
        path_pub=PoseStamped()
        path_pub.pose.position.x = nodes.points[i].x
        path_pub.pose.position.y = nodes.points[i].y
        path_pub.pose.position.z = nodes.points[i].z

        yaw = np.arctan2(nodes.points[i+1].y-nodes.points[i].y,nodes.points[i+1].x-nodes.points[i].x)
        qx,qy,qz,qw = get_quaternion_from_euler(0,0,yaw)
        path_pub.pose.orientation.x = qx
        path_pub.pose.orientation.y = qy
        path_pub.pose.orientation.z = qz
        path_pub.pose.orientation.w = qw
        
        path.poses.append(path_pub)

 
    
    
    if ( (loop_radius.pose.position.x**2) +(loop_radius.pose.position.y**2) <= 1)and(loop_radius.pose.position.y<0):

        # print(path)
        print("loop path generated")
        pub_path.publish(path)
        

        #return path   
    '''
    i = 0
    for points in edges_points:
        marker_est = Marker()
        marker_est.header.frame_id = "map"
        marker_est.ns = "loop_closure_to_edges"+str(i)
        marker_est.id = 42+i
        marker_est.type = Marker.CUBE
        marker_est.action = Marker.ADD
        #marker_est.points = points
        marker_est.text = edges.text
        marker_est.color.r, marker_est.color.g, marker_est.color.b = (0, 255, 0)
        marker_est.color.a = 0.5
        marker_est.scale.x, marker_est.scale.y, marker_est.scale.z = (0.06, 0.06, 0.06)
        pub_marker_array.markers.append(marker_est)
        i+=1
    '''    
    #pub.publish(vel_msg)
    


    
        
if __name__=='__main__':
    # Init ROS
    rospy.init_node('loop_closure', anonymous=True)
    # Subscribers
    
    rospy.Subscriber('/hdl_graph_slam/markers', MarkerArray, callback)
    
    # Publishers
    pub_path = rospy.Publisher('/loop_path', Path, queue_size=1)
    
    rospy.spin()
   
