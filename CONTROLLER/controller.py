import time
import numpy as np

# PID control parameters
pre_t = time.time()
error_arr = np.zeros(5)
brake = 0
def PID(error, p, i, d):
    global pre_t, error_arr 
    # Shift and store error history
    error_arr[1:] = error_arr[:-1]
    error_arr[0] = error 
    # Calculate Proportional term
    P = error * p
    # Calculate delta time
    delta_t = time.time() - pre_t
    pre_t = time.time() 
    # Calculate Integral term
    I = np.sum(error_arr) * delta_t * i
    # Calculate Derivative term (if error_arr[1] exists)
    if delta_t > 0:
        D = (error - error_arr[1]) / delta_t * d
    else:
        D = 0
    # Compute the total PID output
    angle = P + I + D
    # Apply output limit
    if abs(angle) > 30:
        angle = np.sign(angle) * 30
    
    return float(angle)