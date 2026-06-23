import numpy as np

def compute_tracking_errors(x_path,y_path,x_hist,y_hist):
    errors = []
    for x, y in zip(x_hist, y_hist):
        distances = np.sqrt((x_path - x)**2 +(y_path - y)**2)
        
        errors.append(np.min(distances))

    return np.array(errors)

def compute_rms_error(errors):
    return np.sqrt(np.mean(errors**2))

def compute_max_error(errors):
    return np.max(errors)


def compute_mean_error(errors):
    return np.mean(errors)