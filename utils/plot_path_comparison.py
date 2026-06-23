import matplotlib.pyplot as plt

def plot_path(
    path_x, path_y,
    pp_x_hist, pp_y_hist,
    stanley_x_hist, stanley_y_hist,
    lqr_x_hist, lqr_y_hist
):

    plt.figure(figsize=(10, 7))

    plt.plot(
        path_x,
        path_y,
        'k--',
        linewidth=3,
        label='Reference Path'
    )

    plt.plot(
        pp_x_hist,
        pp_y_hist,
        '-',
        linewidth=2,
        label='Pure Pursuit'
    )

    plt.plot(
        stanley_x_hist,
        stanley_y_hist,
        '-.',
        linewidth=2,
        label='Stanley'
    )

    plt.plot(
        lqr_x_hist,
        lqr_y_hist,
        ':',
        linewidth=3,
        label='LQR'
    )

    plt.title("Path Tracking Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")

    plt.axis('equal')
    plt.grid(True)
    plt.legend(fontsize=11)

    plt.show()