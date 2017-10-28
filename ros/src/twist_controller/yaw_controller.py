import math


class YawController(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        """Initialization

        :param wheel_base: The distance (m) between the centers of the
                           front and rear wheels.
        :param steer_ratio: The ratio of the number of degrees of turn
                            of the steering wheel to the number of
                            degrees the wheel turn as a result.
        :param max_lat_accel: Maximum latitude acceleration (m/s^2)
        :param max_steer_angle: Maximum steering angle (deg).
        """
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

    def get_steering(self, cmd_speed, cmd_yaw_rate, current_speed):
        """Get steering angle

        :param cmd_speed: target speed (m/s)
        :param cmd_yaw_rate: target yaw rate (rad/s)
        :param current_speed: current speed (m/s)
        :return: steering angle (deg)
        """
        # scale the yaw rate according to speed
        desired_yaw_rate = cmd_yaw_rate
        if cmd_speed > 0:
            desired_yaw_rate = current_speed*desired_yaw_rate/cmd_speed

        if abs(current_speed) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_speed)
            desired_yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, desired_yaw_rate))

            if abs(desired_yaw_rate) < 0.01:
                return 0.0

            # We do not need a minimum speed as a constraint here since the angle is
            # bound by self.min_angle and self.max_angle
            radius = current_speed / desired_yaw_rate
            return self.get_angle(radius)
        else:
            return 0.0

    def get_angle(self, radius):
        """Get turning angle

        :param radius: turning radius (m)
        :return: steering angle (deg)
        """
        angle = math.atan(self.wheel_base / radius) * self.steer_ratio

        return max(self.min_angle, min(self.max_angle, angle))
