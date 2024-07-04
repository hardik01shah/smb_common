#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from object_detection_msgs.msg import ObjectDetectionInfoArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import time
from tf import TransformListener
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import numpy as np

ELAPSED_TIMEOUT = 120
STUCK_TIMEOUT = 620

BBOX_AREA_THRESHOLD = 50*50
CONFIDENCE_THRESHOLD = 0.2

DETECTED_REGIONS = []

class WaypointMux:
    STATE_MOVING = 0
    STATE_STUCK = 1
    STATE_TIME_OVER = 2
    STATE_OBJECT_DETECTED = 3

    def __init__(self):
        rospy.init_node('waypoint_mux', anonymous=True)
        
        self.initial_position = PointStamped()
        self.current_position = PointStamped()
        self.last_position = PointStamped()
        self.last_significant_movement_time = time.time()
        
        self.initialized = False
        self.start_time = time.time()
        
        self.goalpoint_pub = rospy.Publisher('/goal_point', PointStamped, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/way_point', PointStamped, queue_size=10)
        rospy.Subscriber('/way_point_tare', PointStamped, self.waypoint_tare_callback)
        rospy.Subscriber('/graph_msf/est_odometry_odom_imu', Odometry, self.position_callback)
        
        # Object detection subscriber
        rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, self.obj_callback)
        self.pub_det = rospy.Publisher('/custom/object_poses', PointStamped, queue_size=10)
        self.obj_detected = False
        self.obj_position = PointStamped()
        self.obj_waypoint = PointStamped()
        self.tf = TransformListener()

        self.waypoint_tare = PointStamped()
        self.state = self.STATE_MOVING
        rospy.loginfo("Initialized in STATE_MOVING")
    
    def obj_callback(self, msg):
        # If there is a detection, change the state and follow the object
        for detection in msg.info:
            class_name = detection.class_id
            conf = detection.confidence

            x = detection.position.x
            y = detection.position.y
            z = detection.position.z

            bb_minx = detection.bounding_box_min_x
            bb_miny = detection.bounding_box_min_y
            bb_maxx = detection.bounding_box_max_x
            bb_maxy = detection.bounding_box_max_y

            area = (bb_maxx - bb_minx) * (bb_maxy - bb_miny)

            if area < BBOX_AREA_THRESHOLD or conf < CONFIDENCE_THRESHOLD:
                continue

            # Transform the object pose from the camera_link frame to the odom_graph_msf frame
            pose = PoseStamped()
            pose.header = Header()
            pose.header = msg.header
            pose.pose.position = detection.position
            pose = self.tf.transformPose('odom_graph_msf', pose)
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z

            pub_msg = PointStamped()
            pub_msg.header = pose.header
            assert pub_msg.header.frame_id == 'odom_graph_msf'
            pub_msg.point = pose.pose.position

            for region in DETECTED_REGIONS:
                if self.calculate_distance(region, pub_msg) < 4.0:
                    continue

            # DEBUG
            # Publish the object poses in the odom_graph_msf frame
            self.pub_det.publish(pub_msg)

            self.obj_position = pub_msg
            self.obj_detected = True

        
    def waypoint_tare_callback(self, msg):
        self.waypoint_tare = msg
        # rospy.loginfo(f"Received waypoint tare: {self.waypoint_tare}")
    
    def position_callback(self, msg):
        self.current_position.header.frame_id = msg.header.frame_id
        self.current_position.point.x = msg.pose.pose.position.x
        self.current_position.point.y = msg.pose.pose.position.y
        self.current_position.point.z = msg.pose.pose.position.z

        # rospy.loginfo(f"Current position: {self.current_position}")

        if not self.initialized:
            # self.initial_position = self.current_position
            self.last_position = self.current_position
            self.initialized = True
            # rospy.loginfo(f"Initialized initial position: {self.initial_position}")

        distance_moved = self.calculate_distance(self.current_position, self.last_position)
        if distance_moved >= 0.5:
            self.last_significant_movement_time = time.time()
            self.last_position = self.current_position
            self.state = self.STATE_MOVING
            rospy.loginfo("State changed to STATE_MOVING")

            self.current_position.header.stamp = rospy.Time.now()
            self.goalpoint_pub.publish(self.current_position)
            # rospy.loginfo(f"Robot has moved. Updated last position: {self.last_position}")
    
    def publish_initial_position(self):
        self.initial_position.header.stamp = rospy.Time.now()
        self.initial_position.header.frame_id = 'world_graph_msf'  # Match the frame ID
        # rospy.loginfo(f"Publishing initial position: {self.initial_position}")
        # self.waypoint_pub.publish(self.initial_position)
        self.goalpoint_pub.publish(self.initial_position)
    
    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            elapsed_time = time.time() - self.start_time
            stationary_time = time.time() - self.last_significant_movement_time

            if elapsed_time > ELAPSED_TIMEOUT:
                if self.state != self.STATE_TIME_OVER:
                    self.state = self.STATE_TIME_OVER
                    rospy.loginfo("State changed to STATE_TIME_OVER")
            elif stationary_time > STUCK_TIMEOUT:
                if self.state != self.STATE_STUCK:
                    self.state = self.STATE_STUCK
                    rospy.loginfo("State changed to STATE_STUCK")
            elif self.obj_detected:
                if self.state != self.STATE_OBJECT_DETECTED:
                    self.state = self.STATE_OBJECT_DETECTED
                    rospy.loginfo("State changed to STATE_OBJECT_DETECTED")
            else:
                if self.state != self.STATE_MOVING:
                    self.state = self.STATE_MOVING
                    rospy.loginfo("State changed to STATE_MOVING")

            if self.state == self.STATE_TIME_OVER or self.state == self.STATE_STUCK:
                self.publish_initial_position()
            elif self.state == self.STATE_OBJECT_DETECTED:
                # calculate the distance between the robot and the object
                distance = self.calculate_distance(self.current_position, self.obj_position)
                if distance < 4.0:
                    self.waypoint_tare.header.stamp = rospy.Time.now()
                    # rospy.loginfo(f"Publishing tare waypoint: {self.waypoint_tare}")
                    self.waypoint_pub.publish(self.waypoint_tare)
                    DETECTED_REGIONS.append(self.obj_position)
                
                else:
                    # First get waypoint in the odom_graph_msf frame, then transform it to the world_graph_msf frame
                    # First get waypoint - 4m in front of the object, direction is the same as the object
                    
                    direction_vec = [self.obj_position.point.x - self.current_position.point.x, self.obj_position.point.y - self.current_position.point.y]
                    direction_vec = direction_vec / np.linalg.norm(direction_vec)
                    cur_pos = np.array([self.obj_position.point.x, self.obj_position.point.y])
                    waypoint = cur_pos - 4.0 * direction_vec
                    self.obj_waypoint.point.x = waypoint[0]
                    self.obj_waypoint.point.y = waypoint[1]
                    self.obj_waypoint.point.z = 0.0
                    self.obj_waypoint.header.stamp = self.obj_position.header.stamp
                    self.obj_waypoint.header.frame_id = 'odom_graph_msf'

                    # Transform the object pose from the odom_graph_msf frame to the world_graph_msf frame
                    pose = PoseStamped()
                    pose.header = Header()
                    pose.header = self.obj_waypoint.header
                    pose.pose.position = self.obj_waypoint.point
                    pose = self.tf.transformPose('world_graph_msf', pose)
                    self.obj_waypoint.point = pose.pose.position

                    self.obj_waypoint.header.stamp = rospy.Time.now()
                    # rospy.loginfo(f"Publishing object waypoint: {self.obj_waypoint}")
                    self.waypoint_pub.publish(self.obj_waypoint)
                    self.obj_detected = False

            else:
                self.waypoint_tare.header.stamp = rospy.Time.now()
                # rospy.loginfo(f"Publishing tare waypoint: {self.waypoint_tare}")
                self.waypoint_pub.publish(self.waypoint_tare)

            rate.sleep()
    
    def calculate_distance(self, pos1, pos2):
        return ((pos1.point.x - pos2.point.x)**2 + (pos1.point.y - pos2.point.y)**2 + (pos1.point.z - pos2.point.z)**2)**0.5

if __name__ == '__main__':
    try:
        mux = WaypointMux()
        mux.run()
    except rospy.ROSInterruptException:
        pass
