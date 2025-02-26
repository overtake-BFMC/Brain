import numpy as np
from pykalman import KalmanFilter

class KalmanFilter_LANE:
    
    def __init__(self, dt=0.2):
        self.dt = dt 

        self.kf = KalmanFilter(
            transition_matrices=np.array([[1, dt],  
                                          [0, 1]]),  
            observation_matrices=np.array([[1, 0]]),  
            initial_state_mean=np.array([0, 0]),  
            transition_covariance=np.array([[0.1, 0],  
                                            [0, 0.1]]),
            observation_covariance=np.array([[2]])  
        )

        # pocetna procena stanja i kovarijansa
        self.state_mean = np.array([0, 0])
        self.state_covariance = np.eye(2) * 1000  

    def correct(self, error):
        #azurira stanje Kalmanovog filtera sa novim očitavanjem (merenom greškom). 
        self.state_mean, self.state_covariance = self.kf.filter_update(
            self.state_mean, self.state_covariance, observation=[error]
        )

    def get_filtered_error(self):
        return self.state_mean[0]