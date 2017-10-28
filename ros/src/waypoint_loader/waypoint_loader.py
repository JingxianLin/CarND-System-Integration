#!/usr/bin/env python

import csv
import math

from geometry_msgs.msg import Quaternion
from styx_msgs.msg import Lane, Waypoint
import tf
import rospy


CSV_HEADER = ['x', 'y', 'z', 'yaw']
# MAX_DECEL = 1.0

# the queue size of '/base_waypoints' is one since it is
# always the same
LOOP_RATE = 0.1
PUB_QUEUE_SIZE = 1


class WaypointLoader(object):
    """WaypointLoader class"""
    def __init__(self):
        """Initialization"""
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        # Estimated buff_size = 10902*22*24 = 5,756,256 bytes!
        self.pub = rospy.Publisher(
            '/base_waypoints', Lane, queue_size=PUB_QUEUE_SIZE)

        # target velocity, 40 km/h for the simulator and 10 km/h for on-site
        self.velocity = rospy.get_param('~velocity')  # private param
        self.load_waypoints(rospy.get_param('~path'))  # private param

        rospy.spin()

    def quaternion_from_yaw(self, yaw):
        """Euler to Quaternion"""
        return tf.transformations.quaternion_from_euler(0., 0., yaw)

    def load_waypoints(self, path):
        """Read waypoints from file"""
        try:
            waypoints = []
            with open(path) as wfile:
                reader = csv.DictReader(wfile, CSV_HEADER)
                for wp in reader:
                    p = Waypoint()
                    p.pose.pose.position.x = float(wp['x'])
                    p.pose.pose.position.y = float(wp['y'])
                    p.pose.pose.position.z = float(wp['z'])
                    q = self.quaternion_from_yaw(float(wp['yaw']))
                    p.pose.pose.orientation = Quaternion(*q)
                    p.twist.twist.linear.x = float(self.speed_h2s(self.velocity))

                    waypoints.append(p)

            self.publish(waypoints)
            rospy.loginfo('Waypoint Loaded')
        except IOError:
            rospy.logerr('%s is not a file', path)

    # def decelerate(self, waypoints):
    #     """"""
    #     last = waypoints[-1]
    #     last.twist.twist.linear.x = 0.
    #     for wp in waypoints[:-1][::-1]:
    #         dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
    #         vel = math.sqrt(2 * MAX_DECEL * dist)
    #         if vel < 1.:
    #             vel = 0.
    #         wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
    #
    #     return waypoints

    def publish(self, waypoints):
        """Publish waypoints"""
        while not rospy.is_shutdown():
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = waypoints

            self.pub.publish(lane)
            rospy.sleep(1/LOOP_RATE)

    @staticmethod
    def distance(p1, p2):
        """Distance between two points in 3D space"""
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    @staticmethod
    def speed_h2s(speed):
        """km/h to m/s"""
        return speed/3.6

    @staticmethod
    def speed_s2h(speed):
        """m/s to km/h"""
        return speed*3.6


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
