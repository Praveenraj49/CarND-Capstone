
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
import time
import math


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity,wheel_base , wheel_radius, steer_ratio , min_speed , max_lat_accel,max_steer_angle,
                 decel_limit ,accel_limit, throttle_limit, tor_limit ):
        # TODO: Implement
        self.yaw_controller  = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel,max_steer_angle)
        self.tau = 0.2
        self.ts = 0.2

        self.vel_lpf = LowPassFilter(self.tau, self.ts)
        self.kp = 0.001
        self.ki = 0.0
        self.kd  = 0.01

        self.vehicle_mass = vehicle_mass + GAS_DENSITY*fuel_capacity
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.throttle_limit = throttle_limit
        self.tor_limit = tor_limit
        self.accel_limit = accel_limit
        self.decel_limit  = decel_limit

        self.throttle_controller  = PID(self.kp , self.ki, self.kd , -self.throttle_limit , self.throttle_limit)

        self.last_time  = rospy.get_time()

    def control(self, current_vel , dbw_enabled , linear_vel , angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0,0.0,0.0

        current_vel = self.vel_lpf.filt(current_vel)

        steering  = self.yaw_controller.get_steering(linear_vel , angular_vel , current_vel)

        vel_error  = linear_vel - current_vel

        current_time  = rospy.get_time()
        sample_time  = current_time  - self.last_time
        self.last_time = current_time

        throttle  = self.throttle_controller.step(vel_error , sample_time)
        brake = 0

        if linear_vel ==0.0 and current_vel <0.1:
            throttle = 0
            brake = self.tor_limit #  NM this is mim torque required for Carla Car at idle

        elif throttle <0.1 and vel_error <0:
            throttle = 0
            decel = max (vel_error , self.decel_limit)
            brake  = abs(decel)* self.vehicle_mass * self.wheel_radius

        return throttle , brake , steering
