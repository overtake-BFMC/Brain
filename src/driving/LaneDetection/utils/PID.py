import time

class PIDController:
    def __init__( self, Kp, Ki, Kd ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

        self.last_time = time.time()


    def setParameters( self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def pid_formula( self, error ):

        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        if delta_time <= 0:
            delta_time = 1e-6

        #error P dejstvo
        self.integral += error * delta_time #I dejstvo
        self.integral = max( -50, min( self.integral, 50 ))

        derivative = ( error - self.prev_error )  / delta_time #D dejstvo
        self.prev_error = error 

        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
