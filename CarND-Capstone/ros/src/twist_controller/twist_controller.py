import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deanband, decel_limit, accel_limit, 
                 wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        kp = 0.5
        ki = 0.001
        kd = 0.01
        tau = 0.1 # Cutoff frequency
        ts = 0.02 # Sample time based on 50 Hz
        
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        self.throttle_controller = PID(kp, ki, kd, 0.0, 0.2)
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deanband = brake_deanband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()

    def control(self, vel_cur, dbw_enabled, vel_lin, vel_ang):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = steering = brake = 0.0
        
        if not dbw_enabled:
            self.throttle_controller.reset()
        else:
            vel_cur_filt = self.vel_lpf.filt(vel_cur)
            steering = self.yaw_controller.get_steering(vel_lin, vel_ang, vel_cur_filt)
            
            vel_diff = vel_lin - vel_cur_filt
            
            current_time = rospy.get_time()
            sample_time = current_time - self.last_time
            self.last_time = current_time
            
            throttle = self.throttle_controller.step(vel_diff, sample_time)
            brake = 0.0
            
            if vel_lin == 0.0 and vel_cur_filt < 0.1:
                throttle = 0.0
                brake = 700.0 # Hold the vehicle to static by applying a large braking torque when speed is sufficiently small
            elif throttle < 0.1 and vel_diff < 0:
                throttle = 0.0
                decel = max(vel_diff, self.decel_limit)
                brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, steering, brake
