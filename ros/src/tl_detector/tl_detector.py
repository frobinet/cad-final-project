#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Waypoint
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from TrafficLightDetector import TrafficLightDetector
from waypoint_updater.WaypointsDatabase import WaypointsDatabase
import cv2
import yaml
import numpy as np
import cv2

STATE_COUNT_THRESHOLD = 3
TRAFFIC_LIGHT_VISIBLE_DISTANCE = 75  # Expressed as a number of waypoints

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.bridge = CvBridge()

        self.use_ground_truth = True # TODO Set to False when you don't want to use ground truth traffic light information any longer
        self.detector = TrafficLightDetector()
        self.state = TrafficLight.UNKNOWN
        self.state_count = 0
        self.last_state = TrafficLight.UNKNOWN

        config = yaml.safe_load(rospy.get_param("/traffic_light_config"))
        self.stopline_positions = config['stop_line_positions']

        # Subscribe to input topics
        self.current_pose = None
        self.camera_frame = None
        self.traffic_lights_groundtruth = None
        self.waypoints_db = None
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_callback)
        if self.use_ground_truth:
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_groundtruth_callback)
        rospy.Subscriber('/image_color', Image, self.camera_frame_callback, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.track_waypoints_callback)
                
        # Topic to publish traffic light detections
        # Publish waypoint index of the stop line corresponding to next red light
        # If no red light detected, publish -1  
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoints', Int32, queue_size=1)
        rate = rospy.Rate(2.5) # You can adjust this rate depending on your classifier, but images are published at around ~2-3Hz
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()

    def process(self): 
        # Find closest traffic light stopline
        if self.current_pose is None or self.waypoints_db is None:
            return self.publish_upcoming_light(None, TrafficLight.UNKNOWN)
        dist, closest_light_wp_idx, closest_light_index = self.get_closest_stopline_infos()
        # If next traffic light is close enough, attempt detection
        if dist > TRAFFIC_LIGHT_VISIBLE_DISTANCE:
            return self.publish_upcoming_light(None, TrafficLight.UNKNOWN)
        else:
            if self.traffic_lights_groundtruth is not None:
                state = self.traffic_lights_groundtruth[closest_light_index].state
            elif self.camera_frame is not None:
                state = self.detector.detect_state(self.camera_frame)
            else:
                state = TrafficLight.UNKNOWN
            self.publish_upcoming_light(closest_light_wp_idx, state)
    
    def get_closest_stopline_infos(self):
        # Find the next closest traffic light along the track
        #     - First find the closest waypoint to the car
        #     - Next find the traffic light closest to this waypoint, but coming after it
        _, car_wp_idx = self.waypoints_db.get_next_closest_idx(self.current_pose)
        smallest_dist = len(self.waypoints_db.waypoints)
        for i, line in enumerate(self.stopline_positions): # Traffic light stop lines
            _, line_wp_idx = self.waypoints_db.get_next_closest_idx((line[0], line[1]))
            dist = line_wp_idx - car_wp_idx
            if dist >= 0 and dist < smallest_dist:
                smallest_dist = dist
                closest_light_wp_idx = line_wp_idx
                closest_light_index = i
        return smallest_dist, closest_light_wp_idx, closest_light_index


    def publish_upcoming_light(self, light_wp, state):
        """Publishes the index of the waypoint closest to the red light's stop line to /traffic_waypoint
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is used.
        """
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        self.state_count += 1

    def current_pose_callback(self, msg):
        self.current_pose = np.array([msg.pose.position.x, msg.pose.position.y])

    def traffic_light_groundtruth_callback(self, msg: TrafficLightArray):
        self.traffic_lights_groundtruth = msg.lights

    def camera_frame_callback(self, msg):
        self.camera_frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
    
    def track_waypoints_callback(self, msg: Lane):
        self.waypoints_db = WaypointsDatabase(msg.waypoints)

if __name__ == '__main__':
    try:
        TLDetector()
    except Exception as ex:
        rospy.logerr('Could not start traffic node.')
        raise ex
