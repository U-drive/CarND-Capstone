
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    #def __init__(self, *args, **kwargs):
    def __init__(self, vehicle_mass, fuel_capacity, break_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # Create Yaw Controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # Create PID Controller
        # kp, ki, kd, min, max
        self.kp = ?
        self.ki = ?
        self.kd = ?
        self.speed_controller_pid = PID(self.kp, self.ki, self.kd, decel_limit, accel_limit)

        # Create Low Pass Filter
        # tau, ts
        self.tau = ?
        self.ts = ?
        self.low_pass_filter = LowPassFilter(self.tau, self.ts) 
	

    def control(self, desired_lin_vel, desired_ang_vel, current_vel, dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs
        return throttle, brake, steer
