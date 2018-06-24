#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        # initialise this node
        rospy.init_node('waypoint_updater')

        # define the subscribers and the corresponding callback function
        # /current_pose represents the EGO position
        # /base_waypoints represent
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.ego_pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        # don't need spin(). Replaced by loop().
        # rospy.spin()

        # infinite loop that suspends itself every 20ms
        self.loop()

    def loop(self):
        # publish closest waypoint every 20ms and pause execution

        # define the cycle time of this module
        execution_freq = rospy.Rate(50)  # value taken over from tutorial

        # check if roscore is active
        while not rospy.is_shutdown():
            # asynchronous thread execution: check if variables have been initialized
            # (avoid potential access to undefined variables)
            if self.ego_pose is None or self.base_waypoints is None or self.waypoint_tree is None:
                continue
                # publish closest waypoint
            next_wpt_idx = self.get_next_waypoint_idx()
            self.publish_nxt_waypoints(next_wpt_idx)
            # got to sleep for 20ms
            execution_freq.sleep()

    def get_next_waypoint_idx(self):
        x_coordinate = self.ego_pose.pose.position.x
        y_coordinate = self.ego_pose.pose.position.y
        closest_index = self.waypoint_tree.query([x_coordinate, y_coordinate], 1)[1]
        closest_waypoint_in_2d = self.waypoints_2d[closest_index]
        previous_waypoint_in_2d = self.waypoints_2d[closest_index - 1]
        # define vectors using 2d coordinates
        ego_vector = np.array([x_coordinate, y_coordinate])
        closest_waypoint_vector = np.array(closest_waypoint_in_2d)
        previous_waypoint_vector = np.array(previous_waypoint_in_2d)
        # define intersection with hyperplane
        intersection = np.dot(closest_waypoint_vector - previous_waypoint_vector, ego_vector - closest_waypoint_vector)
        # check if found waypoint is behind the vehicle
        if intersection > 0:
            # use next waypoint
            closest_index = (closest_index + 1) % len(self.waypoints_2d)
        return closest_index

    def pose_cb(self, msg):
        self.ego_pose = msg

    def publish_nxt_waypoints(self, nxt_idx):
        waypoints_on_lane = Lane()
        # copy header
        waypoints_on_lane.header = self.base_waypoints.header
        # get all waypoints starting from closest until closest + 200
        waypoints_on_lane.waypoints = self.base_waypoints.waypoints[nxt_idx:nxt_idx + LOOKAHEAD_WPS]
        # store into final variable
        self.final_waypoints_pub.publish(waypoints_on_lane)

    def waypoints_cb(self, waypoints):
        # copy of lane waypoints
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
