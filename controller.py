import control 
import numpy as np


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
    
        # derivative
        D = self.Kd * (0 - current_v) 
    
        # total output with hover thrust to counteract gravity
        hover_thrust = 9.81 
        return P + I + D + hover_thrust
    
class LQRController:
    def __init__(self, A, B, Q, R ):

        #we only care about K, so we ignore the other two outputs of the lqr function
        self.K, _, _ = control.lqr(A, B, Q, R)
    
    def compute(self, state):
        return np.dot(-self.K, state)
    

