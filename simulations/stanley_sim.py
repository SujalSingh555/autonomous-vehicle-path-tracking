import numpy as np
import paths.generate_path as gp
from vehicle.bicycle_model import BicycleModel
from controllers.pure_pursuit import PurePursuit
from controllers.stanley import Stanley
from utils.metrics import *
from utils.sensor_noise import add_noise


def run_simulation_stanley(path_x, path_y):
    L=4
    v = 10                                
    vehicle=BicycleModel(L=L,v=v)
    
    k=5
    k_psi=1

    controller=Stanley(k,k_psi)

    
    freq=20.0
    dt = 1/freq

    x_hist = []
    y_hist = []

    max_steps=7000

    for i in range(max_steps):

        x, y, psi = vehicle.get_state()

        x_meas, y_meas, psi_meas = add_noise(x,y,psi)
        delta = controller.control(x_meas,y_meas,psi_meas,path_x,path_y,v,L)
        #delta = controller.control(x, y, psi,path_x, path_y,v,L)
        

        vehicle.update(0, delta, dt)

        x_hist.append(x)
        y_hist.append(y)

        goal_dist = np.hypot(
            x - path_x[-1],
            y - path_y[-1]
        )

        if goal_dist < 2:
            break

    return x_hist,y_hist

    



