from simulations.pure_pursuit_sim import run_simulation_pp
from simulations.stanley_sim import run_simulation_stanley
from simulations.lqr_sim import run_simulation_lqr
from utils import plot_path_comparison,animation
from utils import plot_error_comparison
import paths.generate_path as gp
from utils.metrics import *

def main():
    path_x, path_y=gp.generate_lane_change_path()

    stanley_x_hist,stanley_y_hist,debug_data=run_simulation_stanley(path_x, path_y)
    pp_x_hist,pp_y_hist,de=run_simulation_pp(path_x, path_y)
    lqr_x_hist,lqr_y_hist,delta_hist=run_simulation_lqr(path_x,path_y)


    plot_path_comparison.plot_path(path_x,
    path_y,
    pp_x_hist,
    pp_y_hist,
    stanley_x_hist,
    stanley_y_hist,
    lqr_x_hist,
    lqr_y_hist)

    dt=1/20

    plot_error_comparison.plot_path(path_x,
    path_y,
    pp_x_hist,
    pp_y_hist,
    stanley_x_hist,
    stanley_y_hist,lqr_x_hist,lqr_y_hist,dt)

    animation.animate_vehicle(path_x,path_y,stanley_x_hist,stanley_y_hist,debug_data=debug_data,dt=0.05)

    pp_errors = compute_tracking_errors(path_x,path_y,pp_x_hist,pp_y_hist)
    stanley_errors = compute_tracking_errors(path_x,path_y,stanley_x_hist,stanley_y_hist)
    lqr_errors = compute_tracking_errors(path_x,path_y,lqr_x_hist,lqr_y_hist)

    print("\n===== Controller Comparison =====")

    print(f"Mean Error:")
    print(f"  Pure Pursuit : {compute_mean_error(pp_errors):.4f}")
    print(f"  Stanley      : {compute_mean_error(stanley_errors):.4f}")
    print(f"  LQR          : {compute_mean_error(lqr_errors):.4f}")

    print(f"\nRMS Error:")
    print(f"  Pure Pursuit : {compute_rms_error(pp_errors):.4f}")
    print(f"  Stanley      : {compute_rms_error(stanley_errors):.4f}")
    print(f"  LQR          : {compute_rms_error(lqr_errors):.4f}")

    print(f"\nMax Error:")
    print(f"  Pure Pursuit : {compute_max_error(pp_errors):.4f}")
    print(f"  Stanley      : {compute_max_error(stanley_errors):.4f}")
    print(f"  LQR          : {compute_max_error(lqr_errors):.4f}")

if __name__ == "__main__":
    main()