#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from WaypointsDatabase import WaypointsDatabase
import numpy as np

'''
This node will publish waypoints ahead of the car's current position.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribe to input topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.track_waypoints_callback)
        rospy.Subscriber("/traffic_waypoints", Int32, self.next_traffic_light_waypoint_callback)

        # Publisher for computed final waypoints
        self.final_waypoints_publisher = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.N = 100 # Number of waypoints to publish (planning horizon)
        self.max_deceleration = 0.5

        # Wait until the required info has been received
        rate = rospy.Rate(50) # 50Hz loop
        self.current_car_position = None
        self.waypoints_db = None
        self.next_traffic_light_stopline_index = -1
        rospy.loginfo("waiting for initial waypoint")
        while not rospy.is_shutdown():
            if self.current_car_position is not None and self.waypoints_db is not None:
                break
            rate.sleep()
        # Main loop for the node, running at a fixed rate
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()
        
    def pose_callback(self, msg: PoseStamped):
        self.current_car_position = np.array([msg.pose.position.x, msg.pose.position.y]) # XYZ position of the car

    def track_waypoints_callback(self, msg: Lane):
        self.waypoints_db = WaypointsDatabase(msg.waypoints)
    
    def next_traffic_light_waypoint_callback(self, msg: Int32):
        self.next_traffic_light_stopline_index = msg.data
    
    def process(self):
        # TODO: Use self.final_waypoints_pub to publish the next target waypoints
        # In phase 1: we can ignore traffic lights and simply output the next N waypoints *ahead of the car*, with their default velocity
        # In phase 2: you need to adjust target speeds on waypoints in order to smoothly brake until the car reaches the waypoint
        # corresponding to the next red light's stop line (stored in self.next_traffic_light_stopline_index, == -1 if no next traffic light).
        # Advice: make sure to complete dbw_node and have the car driving correctly while ignoring traffic lights before you tackle phase 2 
        msg = Lane()
        self.final_waypoints_publisher.publish(msg)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except Exception as ex:
        rospy.logerr('Could not start waypoint updater node.')
        raise ex
