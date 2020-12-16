#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd
from control.controllers import SteeringAngleController, ThrottleBrakeController

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        deceleration_limit = rospy.get_param('~decel_limit', -5)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steering_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lateral_acceleration = rospy.get_param('~max_lat_accel', 3.)
        max_steering_angle = rospy.get_param('~max_steer_angle', 8.)

        self.throttle_brake_controller = ThrottleBrakeController(vehicle_mass, wheel_radius, deceleration_limit)
        self.steering_controller = SteeringAngleController(wheel_base, steering_ratio, max_steering_angle, max_lateral_acceleration)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # TODO: Subscribe to /twist_cmd to get target linear and angular velocities, and /current_velocity to get the current velocity
        # Keep in mind the following:
        #   - We are assuming that the road is flat, so only angular.z velocity matters
        #   - The coordinate systems is centered on the car, so the linear velocity of the car is linear.x
        # You will need to setup callbacks to refresh the values of the following properties (see waypoint updater for an example)
        self.current_linear_velocity = None
        self.target_linear_velocity = None
        self.target_angular_velocity = None
    
        # Main loop
        rate = rospy.Rate(50) # Running at 50 Hz
        while not rospy.is_shutdown():
            if self.current_linear_velocity != None and self.target_linear_velocity != None and self.target_angular_velocity != None:
                # TODO Fix both controllers and then uncomment
                #throttle, brake = self.throttle_brake_controller.control(self.current_linear_velocity, self.target_linear_velocity) # TODO Fix control method
                #steering_wheel_angle = self.steering_controller.control(self.current_linear_velocity, self.target_linear_velocity, self.target_angular_velocity) # TODO Fix control method
                throttle, brake, steering_wheel_angle = 1., 0., 0.
                self.publish(throttle, brake, steering_wheel_angle)
                rate.sleep()

    def publish(self, throttle: float, brake: float, steer: float):
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
