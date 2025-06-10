import numpy as np

#Kalman filter not localized
class kalmanFilterLocalization:
    def __init__( self, dt ):

        #self.vehicle = vehicle
        self.dt = dt

        # NAPOMENA: korigovati vrednosti matrica ako je potrebno

        # Initial state: x, y, yaw
        self.P = np.eye( 3 ) * 1e-3  # Matrica kovarijansi stanja

        #self.M = np.diag( [0.05, 0.05, 0.01] )  # Matrica kovarijansi suma upravljanja 
        self.M = np.diag([0.05**2, np.deg2rad(1.0)**2])  # Shape: (2, 2)


        self.Q = np.diag( [0.2, 0.2] ) # Matrica kovarijansi suma merenja  

    def prediction(self, dx, v, delta, L):
        
        # iz Path Follow - pozivanje?
        #dx = self.vehicle.kinematicBicycleDynamics( self.state ) # dx = [dx, dy, dyaw]
        #print(f"Prediction dx: {dx}")

        # predicted state
        dx = np.array(dx)

        # Jacobian of motion model
        _, _, yaw = dx
        # v = self.vehicle.speed
        # delta = self.vehicle.steeringAngle
        # L = self.vehicle.wheelbase

        F = np.array( [
            [1, 0, -v * np.sin( yaw ) * self.dt],
            [0, 1,  v * np.cos( yaw ) * self.dt],
            [0, 0, 1]
        ] )

        V = np.array( [  
            [np.cos( yaw ) * self.dt, 0],
            [np.sin( yaw ) * self.dt, 0],
            [np.tan( delta ) / L * self.dt,  v * self.dt / ( L * np.cos( delta )** 2 )]
        ] )

        # Predict covariance
        self.P = F @ self.P @ F.T + V @ self.M @ V.T

        #print(f"Prediction: {self.P}")
        return self.P

    def correction( self, dx, z_measured ):
        # Measurement model: observe x and y directly, not yaw - color sensor gives only x and y of stop line
        H = np.array( [
            [1, 0, 0],
            [0, 1, 0]
        ] )

        # Predicted measurement
        dx = np.array(dx)
        z_predicted = H @ dx
        y = z_measured - z_predicted  # residual error

        # NAPOMENA: korekcija samo pozicije na osnovu pozicije stop linije - orijentacija?

        # Kalman gain
        S = H @ self.P @ H.T + self.Q
        K = self.P @ H.T @ np.linalg.inv( S )

        # Update state and covariance
        correctedState = dx + K @ y
        self.P = ( np.eye( 3 ) - K @ H ) @ self.P

        print(f"Corrected state: {correctedState}")
        return correctedState

    def resetFilter(self):
        self.P = np.eye( 3 ) * 1e-3  # Matrica kovarijansi stanja

        #self.M = np.diag( [0.05, 0.05, 0.01] )  # Matrica kovarijansi suma upravljanja 
        self.M = np.diag([0.05**2, np.deg2rad(1.0)**2])  # Shape: (2, 2)

        self.Q = np.diag( [0.2, 0.2] ) # Matrica kovarijansi suma merenja  
