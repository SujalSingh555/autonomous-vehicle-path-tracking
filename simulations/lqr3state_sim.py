import numpy as np
import paths.generate_path as gp
from vehicle.bicycle_model_lqr_3state import BicycleModelLQR
from controllers.lqr_3state import LQR           # ← lqr_3state
from utils.metrics import *
from utils.sensor_noise import add_noise


def run_simulation_lqr3state(path_x, path_y):
    L=4
    vehicle=BicycleModelLQR(L=L)

    Q=np.diagflat([4,4,1])               # ← 3x3
    R=np.diagflat([20,1])
    max_speed=20
    controller=LQR(wheelbase=L,Q=Q,R=R,max_speed=max_speed)

    freq=10.0
    dt = 1/freq

    x_hist = []
    y_hist = []
    delta_hist = []
    a_hist = []

    max_iterations = 5000
    
    for i in range(max_iterations):
        x,y,psi,v,delta=vehicle.get_state()
        x_meas,y_meas,psi_meas = add_noise(x,y,psi,pos_std=0.15,heading_std=0.03)
        
        delta,a=controller.control(x_meas,y_meas,v,psi_meas,path_x,path_y,dt)  # ← delta not delta_dot
        vehicle.update(a=a,delta=delta,dt=dt)                                    # ← delta not delta_dot
        x_hist.append(x)
        y_hist.append(y)
        delta_hist.append(delta)
        a_hist.append(a)

        goal_dist = np.hypot(
            x - path_x[-1],
            y - path_y[-1]
        )

        if goal_dist < 2:
            break

    return(x_hist,y_hist,delta_hist)