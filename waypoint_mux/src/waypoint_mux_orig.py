#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import time


ELAPSED_TIMEOUT = 100
STUCK_TIMEOUT = 620

class WaypointMux:
    STATE_MOVING = 0
    STATE_STUCK = 1
    STATE_TIME_OVER = 2

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

        self.waypoint_tare = PointStamped()
        self.state = self.STATE_MOVING
        rospy.loginfo("Initialized in STATE_MOVING")
    
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
            else:
                if self.state != self.STATE_MOVING:
                    self.state = self.STATE_MOVING
                    rospy.loginfo("State changed to STATE_MOVING")

            if self.state == self.STATE_TIME_OVER or self.state == self.STATE_STUCK:
                self.publish_initial_position()
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
