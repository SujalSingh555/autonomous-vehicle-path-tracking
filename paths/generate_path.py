import numpy as np

def generate_straight_path(length=200, num_points=500):
    x = np.linspace(0, length, num_points)
    y = np.zeros_like(x)
    return x, y

def generate_circular_path(radius=200, num_points=1000):
    theta = np.linspace(np.pi, 0, num_points)

    x = radius/2 + (radius/2) * np.cos(theta)
    y = (radius/2) * np.sin(theta)

    return x, y

def generate_sine_path(length=200,amplitude=50,frequency=0.1,num_points=1000):
    x = np.linspace(0, length, num_points)
    y = amplitude * np.sin(frequency * x)
    return x, y

def generate_lane_change_path(length=200,lane_width=50,num_points=1000):
    x = np.linspace(0, length, num_points)
    y = lane_width * np.tanh((x - length/2) / 5)-lane_width * np.tanh((0 - length/2) / 5)
    return x, y