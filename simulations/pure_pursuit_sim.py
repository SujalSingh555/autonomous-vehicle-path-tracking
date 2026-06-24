import numpy as np
import paths.generate_path as gp
from vehicle.bicycle_model import BicycleModel
from controllers.pure_pursuit import PurePursuit
from utils.metrics import *
from utils.sensor_noise import add_noise


def run_simulation_pp(path_x, path_y):
    # Generate reference path

    L=4
    # Initialize vehicle
    v = 10
    vehicle = BicycleModel(L=L,v=v)

    # Initialize controller
    controller = PurePursuit(wheelbase=L,lookahead_dist=4)

    
    freq=20.0
    dt = 1/freq

    x_hist = []
    y_hist = []

    path_segments = np.hypot(np.diff(path_x), np.diff(path_y))
    #goal_threshold = max(0.5, np.min(path_segments) * 0.5) if len(path_segments) else 0.5

    max_iterations = 5000
    iteration = 0

    
    while iteration < max_iterations:
        iteration += 1

        x, y, psi = vehicle.get_state()

        x_meas, y_meas, psi_meas = add_noise(x,y,psi)
        # adding noise
        delta = controller.control(x_meas,y_meas,psi_meas,path_x,path_y)
        #delta = controller.control(x,y,psi,path_x,path_y)
        #if iteration < 50:
            #print(delta)
        # Propagate vehicle dynamics
        vehicle.update(0,delta,dt)
        # Save trajectory after the move
        x_hist.append(vehicle.x)
        y_hist.append(vehicle.y)

        goal_dist = np.hypot(
            x - path_x[-1],
            y - path_y[-1]
        )
        if goal_dist<2:
            break
    return x_hist,y_hist