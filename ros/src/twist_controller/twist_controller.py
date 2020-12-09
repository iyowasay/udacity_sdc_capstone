
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, wheel_radius, brake_deadband, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, decel_limit, accel_limit):
        
        # assume the min_speed of the car is 0.1 m/s
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        min_throttle = 0.0
        max_throttle = 0.2 # maximun throttle value
        self.throttle_controller = PID(kp,ki,kd,min_throttle, max_throttle) 

        tau = 0.5 # 1/(2*pi*tau) = cutoff frequency
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass 
        self.fuel_capacity = fuel_capacity
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        # self.decel_limit = decel_limit
        # self.accel_limit = accel_limit

        self.last_time = rospy.get_time()

    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enabled:
        	# to avoid the PID controller accumulating error when DBW is not enabled.
        	self.throttle_controller.reset()
        	return 0.,0.,0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_err = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_err, sample_time)
        brake = 0

        # simple check 
        if linear_vel == 0. and current_vel < 0.1:
        	throttle = 0
        	brake = 400 # N*m, to hold the car in place if we are stopped at a traffic light 
        elif throttle < 0.1 and vel_err < 0:
        	throttle = 0
        	decel = max(vel_err, self.decel_limit)
        	brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m, some physics

        # return 1., 0., 0. # make the car going straight 
        return throttle, brake, steering
