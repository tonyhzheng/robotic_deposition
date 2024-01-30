import numpy as np 

class PID:
    def __init__(self, P = 2.0, I = 0.0, D = 0, max_integrator = 1, min_integrator =-1):
        self.Kp             = P  # proportional gain
        self.Ki             = I  # integral gain
        self.Kd             = D  # derivative gain
 
        self.error_prev     = 0
        self.error          = 0

        self.error_integral          = 0
        self.int_e_max      = max_integrator
        self.int_e_min      = min_integrator

        self.current_value  = 0
        self.dt = 1.0/125

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def setdt(self,dt):
        self.dt = dt

    def set_max_integrator(self,max_integrator):
        self.max_integrator=max_integrator

    def set_min_integrator(self,min_integrator):
        self.min_integrator=min_integrator
 
    def solve(self, reference_value, current_value):
        self.error         = reference_value - current_value
        self.error_integral  = self.error_integral + self.error * self.dt

        if self.error_prev==0:
            self.delta_error = 0
        else:
            self.delta_error = ( self.error - self.error_prev)/self.dt
 
        self.error_integral = np.clip(self.error_integral, self.int_e_min, self.int_e_max)

        P_val  = self.Kp * self.error
        I_val  = self.Ki * self.error_integral
        D_val  = self.Kd * self.delta_error
        # print(reference_value, current_value, P_val,I_val,D_val)
        input_PID = P_val + I_val + D_val
        self.error_prev      = self.error

        return input_PID

    def reset_error_integral(self):
        self.error_integral = 0
        self.error_prev = 0
