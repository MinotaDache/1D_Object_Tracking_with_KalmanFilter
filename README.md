# 1D_Object_Tracking_with_KalmanFilter
This repository implements a 1D object tracking system using a Kalman Filter, applied to different kinds of tracking functions like a sinusoidal motion model. The Kalman Filter is used to estimate the object's position from noisy measurements, demonstrating its effectiveness in sensor fusion and state estimation. 

## Features
- Implements a discrete-time Kalman Filter for position tracking
- Supports a sinusoidal motion model as the real object trajectory
- Includes prediction and update steps for state estimation
- Simulates noisy sensor measurements
- Visualizes real trajectory vs. noisy measurements vs. Kalman filter estimates
## Dependencies

- Python 3.x
- NumPy
- Matplotlib
## Usage
Clone the repository and run:
```bash
python kalman_filter.py
```
## Tracking for Sinusoidal function
![Figure_1](https://github.com/user-attachments/assets/7d2b223f-063a-4eb1-b66a-d40f1131457e)

The script will generate a plot showing the real trajectory, noisy sensor measurements, and Kalman Filter predictions.
## Applications
- Autonomous systems (robot localization, UAV tracking)
- Computer vision (object tracking in videos)
- Sensor fusion (integrating data from multiple sources)

Feel free to contribute, fork, or suggest improvements! 
