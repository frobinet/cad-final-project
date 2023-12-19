
from control.lowpass import LowPassFilter
from control.pid import PID
from math import atan
import rospy


class SteeringAngleController():
    def __init__(self, wheel_base, steering_ratio, max_steering_angle, max_lateral_acc):
        self.wheel_base = wheel_base
        self.steering_ratio = steering_ratio
        self.max_lateral_acc = max_lateral_acc
        self.max_steering_angle = max_steering_angle
    
    def control(self, current_linear_velocity, target_linear_velocity, target_angular_velocity):
        # TODO
        # 1. Compute angular velocity:
        #   - assume angular velocity / linear velocity = target angular velocity / target linear velocity
        #   - make sure to cap angular velocity to avoid exceeeding max lateral acceleration (clip angular velocity to [-max_angular_vel, max_angular_vel])
        #   - Compute max allowed angular velocity from given max allowed lateral acceleration using uniform circular motion equations 
        # 2. Compute the vehicle steering angle:
        #   - if angular velocity is 0, the angle should also be 0
        #   - compute the turning radius from angular velocity using circular motion equations
        #   - use the bicycle model equations to compute the steering angle corresponding to the turning radius
        # 3. Compute the *steering wheel* angle:
        #   - steering wheel angle = steering angle * steering ratio
        #   - clip the steering wheel angle to [-max steering wheel angle, max steering wheel angle]
        return 0.0


class ThrottleBrakeController():
    def __init__(self, car_mass, wheel_radius, deceleration_limit):
        self.lowpass_filter = LowPassFilter(tau=0.5, ts=.02)
        self.car_mass = car_mass
        self.deceleration_limit = deceleration_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()

        # Note: you may need to tune the PID parameters Kp, Ki and Kd to reach optimal behavior, 
        # but you may start from the given initial values and only tune them near the end of the project.
        self.pid = PID(kp=0.3, ki=0.1, kd=0, mn=0, mx=0.7)


    def control(self, current_speed, target_speed):
        current_speed = self.lowpass_filter.filter(current_speed) # Use LowPass filter to filter noise from velocity, don't change this
        current_time = rospy.get_time()

        # TODO Step 1: Use the provided PID controller self.pid here to control the throttle (can leave the PID params unchanged at the start, and tune them a bit more when everything else works)
        # TODO Step 2: After predicting throttle, you can handle braking as a postprocessing step
        #   - The car has an automatic transmission, so if target velocity is 0 and current linear velocity is small or zero, you should apply brake (Need ~700Nm brake so the car won't move when it's currently at speed 0)
        #   - don't brake and throttle at the same time! 
        #   - Apply brake if throttle is already small, but you need to slow down
        #   - In order to get a smooth ride, you should also not decelerate more than the given deceleration limit.
        # For now, this code just blindly accelerates!
        throttle, brake = 1.0, 0.0
        return throttle, brake 
