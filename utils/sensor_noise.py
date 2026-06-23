import numpy as np

def add_noise(x, y, psi,pos_std=0.15,heading_std=0.03):

    noisy_x = x + np.random.normal(0, pos_std)
    noisy_y = y + np.random.normal(0, pos_std)
    noisy_psi = psi + np.random.normal(0, heading_std)

    return noisy_x, noisy_y, noisy_psi