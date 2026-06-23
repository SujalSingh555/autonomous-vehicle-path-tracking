import matplotlib.pyplot as plt
from utils.metrics import *

def plot_path(path_x, path_y,
              pp_x_hist, pp_y_hist,
              stanley_x_hist, stanley_y_hist,
              lqr_x_hist, lqr_y_hist,
              dt):

    plt.figure(figsize=(8,6))

    pp_crosstrack_errors = compute_tracking_errors(
        path_x, path_y, pp_x_hist, pp_y_hist
    )

    stanley_crosstrack_errors = compute_tracking_errors(
        path_x, path_y, stanley_x_hist, stanley_y_hist
    )

    lqr_crosstrack_errors = compute_tracking_errors(
        path_x, path_y, lqr_x_hist, lqr_y_hist
    )

    pp_time = [i * dt for i in range(len(pp_crosstrack_errors))]
    stanley_time = [i * dt for i in range(len(stanley_crosstrack_errors))]
    lqr_time = [i * dt for i in range(len(lqr_crosstrack_errors))]

    step = 10

    plt.plot(pp_time[::step],
             pp_crosstrack_errors[::step],
             label='Pure Pursuit')

    plt.plot(stanley_time[::step],
             stanley_crosstrack_errors[::step],
             label='Stanley')

    plt.plot(lqr_time[::step],
             lqr_crosstrack_errors[::step],
             label='LQR')

    plt.title("Cross Track Error Comparison")
    plt.xlabel("Time (s)")
    plt.ylabel("Cross Track Error (m)")

    plt.grid(True)
    plt.legend()

    plt.show()