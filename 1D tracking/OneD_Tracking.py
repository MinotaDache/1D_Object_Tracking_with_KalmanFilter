# 1D object Tracking with Kalman Filter
import numpy as np
import matplotlib.pyplot as plt 


class KalmanFilter(object):

    # we initialize the kalman filter with the following parameters:
    # dt = time step
    # u = control input
    # sda = standard deviation of acceleration
    # sdm = standard deviation of measurement
    def __init__(self, dt, u, sda, sdm):
        self.dt = dt
        self.u = u
        self.sda = sda
        self.A = np.matrix([[1, self.dt], [0, 1]])
        self.B = np.matrix([[(self.dt**2) / 2], [self.dt]])
        self.H = np.matrix([[1, 0]])
        self.Q = (
            np.matrix(
                [[(self.dt**4) / 4, self.dt**3 / 2], [self.dt**3 / 2, self.dt**2]]
            )
            * self.sda**2
        )
        self.R = sdm**2
        self.P = np.eye(self.A.shape[1])
        self.x = np.matrix([[0], [0]])

    def predict(self):

        # Predict the state
        # x = Ax + Bu
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Predict the error covariance
        # P = APAT + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x
    def update(self, z):
        # Update the state with the measurement
        # S = H*P*H' + R
        S = np.dot( self.H,np.dot(self.P,self.H.T)) + self.R 

        # compute the Kalman gain
        # K = PH'S^-1
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S)) # Eq. 11

        # Update the state
        # x = x + K(z - Hx)
        self.x= np.round(self.x + np.dot(K,(z - np.dot(self.H, self.x)))) # Eq. 12

        # Update the error covariance
        # P = (I - KH)P
        I = np.eye(round(self.H.shape [1]))
        self.P= np.dot((I -np.dot(K,self.H)), self.P)# Eq. 13
        return self.x
        # Update the error covariance

    
def main():
    dt = 0.1
    t = np.arange(0, 100, dt)

    # define the modle track
    #real_track = 0.1*((t**2) - t)
    A= 10
    phi = 0
    f= 0.1
    real_track = A* np.sin(2*np.pi*f*t + phi) # the actual position of the object

    u = 2
    sda= 0.25 # standard deviation of acceleration in m/s^2
    sdm = 1.2 # standard deviation of measurement in m 

    # create the Kalman filter object
    kf = KalmanFilter(dt, u, sda, sdm)
    # create the measurements
    predictions = [] # used to store the predictions
    measurements = []


# real_track is actual postion of the object
# the for loop itrates over the time steps/acual position x
    for x in real_track: 
        # measure the position
        # z = Hx + w
        # w is the noise, is the gaussian noise with mean 0 and standard deviation 50 
        # z is the measurement
        # x is the actual position
        # Simulates a noisy sensor reading
        #measure
        z= kf.H*x + np.random.normal(0, 50)  #w=(1.2) simulate the real world sensor noise in measurements  
        # append the measurement to the list
        measurements.append(z.item(0)) # extracts scalar value from the matrix 
        predictions.append(kf.predict()[0]) # uses system model to predict the next state
        # update the state
        kf.update(z.item(0)) # update the state using the measurement
        
    # plot the results
    fig = plt.figure()
    fig.suptitle("Kalman Filer for 1D Object Tracking", fontsize = 20)
    plt.plot(t, measurements, label = "Measurements", color = 'b', linewidth = 0.5)
    plt.plot(t, np.array(real_track), label = "Real Track", color = 'y', linewidth =1.5)
    plt.plot(t, np.squeeze(predictions), label= "Kalman Filter Predictions", color ='r', linewidth = 1.5)

    plt.xlabel("Time (s)", fontsize = 20)
    plt.ylabel("Position (m)", fontsize = 20)
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()
# The code implements a Kalman filter for 1D object tracking.
# The Kalman filter is initialized with parameters such as time step, control input,
# standard deviation of acceleration, and standard deviation of measurement.
# The filter predicts the state and updates it based on noisy measurements.
# The main function simulates the tracking process, generates noisy measurements,
# and visualizes the results using Matplotlib.
# The plot shows the measurements, real track, and Kalman filter predictions.
# The Kalman filter effectively estimates the object's position despite the noise in measurements.
# The Kalman filter is a recursive algorithm that estimates the state of a dynamic system
# from a series of noisy measurements. It is widely used in various applications,
# including robotics, computer vision, and finance.







    
