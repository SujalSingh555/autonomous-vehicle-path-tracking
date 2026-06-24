import numpy as np
import paths.generate_path as gp
from vehicle.bicycle_model import BicycleModel
from controllers.pure_pursuit import PurePursuit
from controllers.stanley import Stanley
from utils.metrics import *
from utils.sensor_noise import add_noise
from utils.kalman_filter import KalmanFilter


def run_simulation_stanley(path_x, path_y):
    L=4
    v = 10                                
    vehicle=BicycleModel(L=L,v=v)
    
    k=12
    k_psi=1

    controller=Stanley(k,k_psi)

    
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
    front_x_hist = []
    front_y_hist = []

    nearest_x_hist = []
    nearest_y_hist = []

    path_heading_hist = []
    cte_hist = []
    debug_data={}
    delta_prev=0

    max_steps=5000

    for i in range(max_steps):

        x, y, psi = vehicle.get_state()

        x_meas, y_meas, psi_meas = add_noise(x,y,psi)

        kf.predict(
            v=v,
            delta=delta_prev,
            L=L
        )

        x_est, y_est, psi_est = kf.update(
            x_meas,
            y_meas,
            psi_meas
        )     

        delta,debug= controller.control(x_est,y_est,psi_est,path_x,path_y,v,L)
        #delta,debug= controller.control(x_meas,y_meas,psi_meas,path_x,path_y,v,L)
        #delta = controller.control(x, y, psi,path_x, path_y,v,L)
        delta_prev=delta
        front_x_hist.append(debug["front_x"])
        front_y_hist.append(debug["front_y"])

        nearest_x_hist.append(debug["nearest_x"])
        nearest_y_hist.append(debug["nearest_y"])

        path_heading_hist.append(debug["path_heading"])

        cte_hist.append(debug["cte"])
                

        vehicle.update(0, delta, dt)

        x_hist.append(x)
        y_hist.append(y)

        goal_dist = np.hypot(
            x - path_x[-1],
            y - path_y[-1]
        )

        if goal_dist < 2:
            break
    debug_data = {
    "front_x": front_x_hist,
    "front_y": front_y_hist,

    "nearest_x": nearest_x_hist,
    "nearest_y": nearest_y_hist,

    "path_heading": path_heading_hist,

    "cte": cte_hist}

    return (x_hist,y_hist,debug_data)

    



