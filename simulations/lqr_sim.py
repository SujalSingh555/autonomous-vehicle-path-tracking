import numpy as np
import paths.generate_path as gp
from vehicle.bicycle_model_lqr import BicycleModelLQR
from controllers.lqr import LQR
from utils.metrics import *
from utils.sensor_noise import add_noise
from utils.kalman_filter import KalmanFilter



def run_simulation_lqr(path_x, path_y):
    L=4
    vehicle=BicycleModelLQR(L=L)

    Q=np.diagflat([10,1,10,1,4])
    R=np.diagflat([50,1])
    max_speed=10
    controller=LQR(wheelbase=L,Q=Q,R=R,max_speed=max_speed)

    freq=20.0
    dt = 1/freq
    kf = KalmanFilter(dt)
    kf.x = np.array([
    [vehicle.x],
    [vehicle.y],
    [vehicle.psi]
    ])

    x_hist = []
    y_hist = []
    delta_hist = []
    a_hist = []

    path_segments = np.hypot(np.diff(path_x), np.diff(path_y))

    max_iterations = 5000
    
    for i in range(max_iterations):
        x,y,psi,v,delta=vehicle.get_state()
        x_meas,y_meas,psi_meas = add_noise(x,y,psi,pos_std=0.15,heading_std=0.03)
        kf.predict(
            v=v,
            delta=delta,
            L=L
        )

        # Kalman update
        x_est,y_est,psi_est = kf.update(
            x_meas,
            y_meas,
            psi_meas
        )
        delta_dot,a=controller.control(x_est,y_est,v,psi_est,path_x,path_y,dt)        
        #delta_dot,a=controller.control(x_meas,y_meas,v,psi_meas,path_x,path_y,dt)
        #delta_dot,a=controller.control(x,y,v,psi,path_x,path_y,dt)
        vehicle.update(a=a,delta_dot=delta_dot,dt=dt)
        x_hist.append(x)
        y_hist.append(y)
        delta_hist.append(delta)
        a_hist.append(a)

        goal_dist = np.hypot(
            x - path_x[-1],
            y - path_y[-1]
        )

        if goal_dist < max_speed*1/30:
            break


    return(x_hist,y_hist,delta_hist)
