
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    #def __init__(self, *args, **kwargs):
    def __init__(self, vehicle_mass, fuel_capacity, break_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
	
	# Create Yaw Controller
	yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

	# Create PID Controller
	# kp, ki, kd, min, max
	pid_controller = PID(?, ?, ?, decel_limit, accel_limit)

	# Create Low Pass Filter
	# tau, ts
	low_pass_filter = LowPassFilter(?, ?) 
	
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
