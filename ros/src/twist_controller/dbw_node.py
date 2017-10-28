#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
LOOP_RATE = 10
PUB_QUEUE_SIZE = 100
SUB_QUEUE_SIZE = 100


class DBWNode(object):
    def __init__(self):
        """Initialization"""
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher(
            '/vehicle/steering_cmd', SteeringCmd, queue_size=PUB_QUEUE_SIZE)
        self.throttle_pub = rospy.Publisher(
            '/vehicle/throttle_cmd', ThrottleCmd, queue_size=PUB_QUEUE_SIZE)
        self.brake_pub = rospy.Publisher(
            '/vehicle/brake_cmd', BrakeCmd, queue_size=PUB_QUEUE_SIZE)

        self.yaw_controller = YawController(
            wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        self.throttle_filter = LowPassFilter(0.1, 0.9)
        self.throttle_controller = PID(1., 0., 0., 0, accel_limit)

        self._dbw_enabled = True
        self.dbw_enabled = True
        self.current_speed = 0.0  # current speed
        self.cmd_speed = 0.0  # target speed
        self.cmd_yaw_rate = 0.0  # target yaw rate at the target speed

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb,
                         queue_size=SUB_QUEUE_SIZE)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb,
                         queue_size=SUB_QUEUE_SIZE)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb,
                         queue_size=SUB_QUEUE_SIZE)

        self.loop()

    @property
    def dbw_enable(self):
        """getter"""
        return self._dbw_enabled

    @dbw_enable.setter
    def dbw_enable(self, value):
        """setter"""
        if self._dbw_enabled is True and value is False:
            print("Self-driving mode off")
        elif self._dbw_enabled is False and value is True:
            print("Self-driving mode on")
        else:
            pass

        self._dbw_enabled = value

    def twist_cmd_cb(self, msg):
        """Get target speed and yaw rate"""
        self.cmd_speed = msg.twist.linear.x
        self.cmd_yaw_rate = msg.twist.angular.z
        rospy.loginfo("dbw_node: cmd_speed: {}, cmd_yaw_rate: {}".
                      format(self.cmd_speed, self.cmd_yaw_rate))

    def velocity_cb(self, msg):
        """Get current speed"""
        self.current_speed = msg.twist.linear.x
        # rospy.loginfo("dbw_node: current_speed: {}".format(self.current_speed))

    def dbw_enabled_cb(self, msg):
        """
        :param msg:
        :return:
        """
        # TODO: we do not have this msg!
        self.dbw_enabled = msg.data

    def loop(self):
        """"""
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            if self.dbw_enabled is True:
                # Lock the values in a loop
                current_speed = self.current_speed
                cmd_yaw_rate = self.cmd_yaw_rate
                cmd_speed = self.cmd_speed

                brake = 0
                throttle = self.throttle_controller.step(
                    self.throttle_filter.filt(cmd_speed - current_speed), 1.)

                steering = self.yaw_controller.get_steering(cmd_speed, cmd_yaw_rate, current_speed)

                self.publish(throttle, brake, steering)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        """Publish throttle, brake and steer"""
        rospy.loginfo("dbw_node: Throttle: {}, brake: {}, steering {}"
                      .format(throttle, brake, steer))

        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
