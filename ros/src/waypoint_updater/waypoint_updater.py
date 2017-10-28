#!/usr/bin/env python
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from helper_function import dist_two_points, closest_waypoint

import math
import copy


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
SUB_QUEUE_SIZE = 100
PUB_QUEUE_SIZE = 100


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Estimated buff_size = 10902*22*24 = 5,756,256 bytes!
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb,
                         queue_size=1, buff_size=6000000)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb,
                         queue_size=SUB_QUEUE_SIZE)
        rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb,
                         queue_size=SUB_QUEUE_SIZE)

        # Only waypoint_follower subscribes this msg.
        # Estimated buff_size 200*22*24 = 105,600 bytes, a queue size of 100
        # means 10,560,000 bytes
        self.final_waypoints_pub = rospy.Publisher(
            '/final_waypoints', Lane, queue_size=PUB_QUEUE_SIZE)

        self.prev_nearest_waypoint = [None, None]
        self.map_x = []
        self.map_y = []
        self.map_velocity = []
        self._is_initialized = False

        rospy.spin()

    def pose_cb(self, msg):
        """Publish waypoints"""
        # rospy.loginfo('WaypointUpdater: Current Pose returned.')

        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y

        # self.map_x and self.map_y could be updated during the execution
        # of the method
        map_x = copy.copy(self.map_x)
        map_y = copy.copy(self.map_y)
        map_velocity = copy.copy(self.map_velocity)

        if len(map_x) < LOOKAHEAD_WPS or len(map_y) < LOOKAHEAD_WPS:
            return

        nearest_waypoint = closest_waypoint(pose_x, pose_y, map_x, map_y)
        if not nearest_waypoint:
            return

        # Reduce the frequency of publication
        if self.prev_nearest_waypoint and \
                map_x[nearest_waypoint] == self.prev_nearest_waypoint[0] and \
                map_y[nearest_waypoint] == self.prev_nearest_waypoint[1]:
            return

        # Look for next waypoint - LOOKAHEAD_WPS
        publish_list = []
        for i in range(LOOKAHEAD_WPS):
            try:
                wp = Waypoint()
                wp.pose.pose.position.x = float(map_x[nearest_waypoint+i])
                wp.pose.pose.position.y = float(map_y[nearest_waypoint+i])
                wp.pose.pose.position.z = float(0.0)
                wp.twist.twist.linear.x = float(map_velocity[nearest_waypoint+i])
                publish_list.append(wp)
            except IndexError:
                return

        # If enough waypoints are received, update previous nearest waypoint
        self.prev_nearest_waypoint[0] = map_x[nearest_waypoint]
        self.prev_nearest_waypoint[1] = map_y[nearest_waypoint]
        rospy.loginfo('WaypointUpdater: at waypoint {:.2f}, {:.2f}'.
                      format(self.prev_nearest_waypoint[0],
                             self.prev_nearest_waypoint[1]))

        lane = Lane()
        lane.header.frame_id = msg.header.frame_id
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = publish_list

        self.final_waypoints_pub.publish(lane)
        rospy.loginfo("WaypointUpdater: {} waypoints published!"
                      .format(len(publish_list)))

    def waypoints_cb(self, msg):
        """Create two lists one with X and other with Y waypoints

            Only read the '/base_waypoints' once!
        """
        if self._is_initialized is False:
            # TODO: Is it necessary to add a sanity check here?
            self.map_x = []
            self.map_y = []
            self.map_velocity = []
            for waypoint in msg.waypoints:
                self.map_x.append(waypoint.pose.pose.position.x)
                self.map_y.append(waypoint.pose.pose.position.y)
                self.map_velocity.append(waypoint.twist.twist.linear.x)

            self._is_initialized = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo('WaypointUpdater: traffic waypoint returned')

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        rospy.loginfo('WaypointUpdater: obstacle waypoints returned')

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
