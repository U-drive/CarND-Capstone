
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
        self.throttle_controller_pid = PID(self.kp, self.ki, self.kd, decel_limit, accel_limit)

        # Create Low Pass Filter
        # tau, ts
        self.tau = ?
        self.ts = ?
        self.low_pass_filter = LowPassFilter(self.tau, self.ts) 
	
		# Car Total Mass including Fuel
		# Should we not calculate this outside the constructor to compensate for used fuel?
		# does fuel_capacity just provide total, or current fuel?
		self.total_mass = vehicle_mass + (fuel_capacity * GAS_DENSITY)
		# Breaking force
		self.breaking_force = total_mass * wheel_radius

		# Init Time
		self.current_time = rospy.get_time()
		

    def control(self, desired_lin_vel, desired_ang_vel, current_vel, dbw_status):
        
		# Calculate delta time
		self.delta_time = rospy.get_time() - current_time
		self.current_time = rospy.get_time()

		# Calculate Throttle Value
		# Note that throttle values passed to publish should be in the range 0 to 1.
		self.vel_error = current_vel - desired_lin_vel
		self.new_acceleration_value = self.throttle_controller_pid.step(self.vel_error, self.delta_time)
		self.filtered_acceleration_value = self.low_pass_filter.filt(self.new_acceleration_value)
		
		if self.filtered_acceleration_value > 0:
			throttle = self.filtered_acceleration_value
			# when accelerating make sure we are not breaking
			brake = 0
		
		# Calculate Break Value
		# Break values passed to publish should be in units of torque (N*m)
		# The correct values of break can be computed using the desired acceleration, weight
		# of the vehicle, and wheel radius.
		if self.filtered_acceleration_value <= 0:
			# only start breaking if not in the break deadband so that the car can be in a state 
			# whereby its slowing down without having to apply the breaks for every deceleration
			if abs(throttle) > break_deadband:
				brake = abs(throttle) * self.breaking_force
				# when breaking make sure we are not accelerating
				throttle = 0

		# Calculate Steer Value
		steer = self.yaw_controller.get_steering_calculated(desired_lin_vel, desired_ang_vel, current_vel)

        return throttle, brake, steer
