class PidControl(object):
    """Pid controller for one direction, f*** Justin Beiber..."""
    def __init__(self, kp, ki, kd, i_max=None):
        #Proporional gain tuning parameter
        self.kp = float(kp)
        #Integral gain, tuning parameter
        self.ki = float(ki)
        #Derivative gain, tuning paramter
        self.kd = float(kd)
        #Current integral calculation
        self.integral = 0.0
        #Previous error, used in online calculation
        self.previous_error = 0.0
        #Maximum value self.integral can take
        self._i_max = i_max
        
    def reset(self):
        """Reset integral and previous error"""
        self.integral = 0.0
        self.previous_error = 0.0
        
    def __call__(self, pv, sp, dt, freeze_ff=False):
        """Update the PidControl with the process value, desired setpoint
        and the time difference. The freeze argument can be used to halt
        the integrator if necessary, by default it is false"""        
        error = sp - pv
        if not freeze_ff:
            #If freeze is True, use the value from last time. This enables
            #callers to decide that the PID controller can not increase
            #above a certain value
            self.integral += error * dt
        if self._i_max is not None:
            self.integral = max(-self._i_max, min(self.integral, self._i_max))
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative
