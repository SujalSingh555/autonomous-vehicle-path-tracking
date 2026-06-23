import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def animate_vehicle(path_x, path_y, x_hist, y_hist, psi_hist=None, dt=0.05, speed_ratio=1.0):
    """
    Simulates vehicle trajectory.
    
    Parameters:
    - dt: Time step between data points in seconds.
    - speed_ratio: 1.0 for real-time, 2.0 for twice as fast, 0.5 for half speed.
    """
    # 1. Calculate the ideal refresh interval in milliseconds
    # Real-time interval would be dt * 1000. We divide by speed_ratio to speed it up.
    target_interval = (dt * 1000) / speed_ratio
    
    # 2. Matplotlib struggles with intervals below ~15-20ms.
    # If the interval is too small, we downsample the data (skip frames) to keep up.
    step = 1
    if target_interval < 20:
        step = int(ceil(20 / target_interval))  # Calculate downsample step
        target_interval = target_interval * step # Recalculate adjusted interval
        
        # Downsample the history arrays
        x_hist = x_hist[::step]
        y_hist = y_hist[::step]
        if psi_hist is not None:
            psi_hist = psi_hist[::step]

    fig, ax = plt.subplots(figsize=(8, 6))

    # Reference path
    ax.plot(path_x, path_y, 'b--', linewidth=2, label='Reference Path')
    
    # Vehicle trajectory & position
    traj_line, = ax.plot([], [], 'r-', linewidth=2, label='Vehicle Trajectory')
    vehicle, = ax.plot([], [], 'ko', markersize=8)
    heading_arrow = None

    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()

    margin = 2
    ax.set_xlim(min(min(path_x), min(x_hist)) - margin, max(max(path_x), max(x_hist)) + margin)
    ax.set_ylim(min(min(path_y), min(y_hist)) - margin, max(max(path_y), max(y_hist)) + margin)

    def update(frame):
        nonlocal heading_arrow

        # Fast array slicing up to the current frame
        traj_line.set_data(x_hist[:frame + 1], y_hist[:frame + 1])
        vehicle.set_data([x_hist[frame]], [y_hist[frame]])

        artists = [traj_line, vehicle]

        if psi_hist is not None:
            if heading_arrow is not None:
                heading_arrow.remove()

            L = 1.5
            dx = L * np.cos(psi_hist[frame])
            dy = L * np.sin(psi_hist[frame])

            heading_arrow = ax.arrow(
                x_hist[frame], y_hist[frame], dx, dy, width=0.05, color='black'
            )
            artists.append(heading_arrow)

        return artists

    ani = FuncAnimation(
        fig,
        update,
        frames=len(x_hist),
        interval=max(int(target_interval), 1), # Ensure interval is at least 1ms
        blit=False,
        repeat=False
    )

    plt.show()
    return ani