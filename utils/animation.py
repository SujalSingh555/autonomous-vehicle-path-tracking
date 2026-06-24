import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import ceil


def animate_vehicle(
    path_x,
    path_y,
    x_hist,
    y_hist,
    psi_hist=None,
    debug_data=None,
    dt=0.05,
    speed_ratio=1.0
):

    target_interval = (dt * 1000) / speed_ratio

    step = 1

    if target_interval < 20:

        step = int(ceil(20 / target_interval))

        target_interval *= step

        x_hist = x_hist[::step]
        y_hist = y_hist[::step]

        if psi_hist is not None:
            psi_hist = psi_hist[::step]

        if debug_data is not None:
            for key in debug_data:
                debug_data[key] = debug_data[key][::step]

    fig, ax = plt.subplots(figsize=(8,6))

    ax.plot(
        path_x,
        path_y,
        'b--',
        linewidth=2,
        label='Reference Path'
    )

    traj_line, = ax.plot(
        [],
        [],
        'r-',
        linewidth=2,
        label='Vehicle Trajectory'
    )

    vehicle, = ax.plot(
        [],
        [],
        'ko',
        markersize=8,
        label='Vehicle'
    )

    heading_arrow = None
    path_arrow = None

    # PURE PURSUIT
    lookahead_point = None
    lookahead_line = None

    # STANLEY
    nearest_point = None
    cte_line = None

    if debug_data is not None:

        if (
            "lookahead_x" in debug_data and
            "lookahead_y" in debug_data
        ):

            lookahead_point, = ax.plot(
                [],
                [],
                'go',
                markersize=8,
                label='Lookahead Point'
            )

            lookahead_line, = ax.plot(
                [],
                [],
                'g:',
                linewidth=1.5
            )

        if (
            "front_x" in debug_data and
            "nearest_x" in debug_data
        ):

            nearest_point, = ax.plot(
                [],
                [],
                'mo',
                markersize=8,
                label='Nearest Path Point'
            )

            cte_line, = ax.plot(
                [],
                [],
                'm--',
                linewidth=2,
                label='Cross Track Error'
            )

    ax.set_aspect('equal')
    ax.grid(True)

    ax.legend(
        loc='upper left',
        bbox_to_anchor=(1.02,1.0)
    )

    margin = 2

    ax.set_xlim(
        min(min(path_x),min(x_hist))-margin,
        max(max(path_x),max(x_hist))+margin
    )

    ax.set_ylim(
        min(min(path_y),min(y_hist))-margin,
        max(max(path_y),max(y_hist))+margin
    )

    def update(frame):

        nonlocal heading_arrow
        nonlocal path_arrow

        traj_line.set_data(
            x_hist[:frame+1],
            y_hist[:frame+1]
        )

        vehicle.set_data(
            [x_hist[frame]],
            [y_hist[frame]]
        )

        artists = [
            traj_line,
            vehicle
        ]

        # Vehicle heading
        if psi_hist is not None:

            if heading_arrow is not None:
                heading_arrow.remove()

            heading_arrow = ax.arrow(
                x_hist[frame],
                y_hist[frame],
                1.5*np.cos(psi_hist[frame]),
                1.5*np.sin(psi_hist[frame]),
                width=0.05,
                color='black'
            )

            artists.append(
                heading_arrow
            )

        # --------------------
        # PURE PURSUIT
        # --------------------

        if (
            debug_data is not None
            and lookahead_point is not None
        ):

            lx = debug_data["lookahead_x"][frame]
            ly = debug_data["lookahead_y"][frame]

            lookahead_point.set_data(
                [lx],
                [ly]
            )

            lookahead_line.set_data(
                [x_hist[frame], lx],
                [y_hist[frame], ly]
            )

            artists.append(
                lookahead_point
            )

            artists.append(
                lookahead_line
            )

        # --------------------
        # STANLEY
        # --------------------

        if (
            debug_data is not None
            and nearest_point is not None
        ):

            fx = debug_data["front_x"][frame]
            fy = debug_data["front_y"][frame]

            nx = debug_data["nearest_x"][frame]
            ny = debug_data["nearest_y"][frame]

            nearest_point.set_data(
                [nx],
                [ny]
            )

            cte_line.set_data(
                [fx, nx],
                [fy, ny]
            )

            if path_arrow is not None:
                path_arrow.remove()

            heading = debug_data["path_heading"][frame]

            path_arrow = ax.arrow(
                nx,
                ny,
                1.5*np.cos(heading),
                1.5*np.sin(heading),
                width=0.05,
                color='blue'
            )

            artists.append(
                nearest_point
            )

            artists.append(
                cte_line
            )

            artists.append(
                path_arrow
            )

        return artists

    ani = FuncAnimation(
        fig,
        update,
        frames=len(x_hist),
        interval=max(
            int(target_interval),
            1
        ),
        repeat=False,
        blit=False
    )

    plt.tight_layout()
    plt.show()

    return ani