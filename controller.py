class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0
    
    def compute(self, target_z, current_z, current_v, dt):
        error = target_z - current_z
    
        # Proportional
        P = self.Kp * error
    
        # Integral
        self.integral += error * dt
        I = self.Ki * self.integral
    
        # Derivative (using velocity directly is cleaner)
        D = self.Kd * (0 - current_v) 
    
        # Total Output + FEED FORWARD (The thrust needed to hover)
        hover_thrust = 9.81 
        return P + I + D + hover_thrust

