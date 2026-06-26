# Autonomous Vehicle Path Tracking Simulator

A Python-based path tracking simulator implementing and comparing **Pure Pursuit**, **Stanley**, and **LQR** controllers on a kinematic bicycle model. Includes sensor noise modelling and Kalman Filter state estimation.

---

## Project Structure

```
├── controllers/
│   ├── pure_pursuit.py
│   ├── stanley.py
│   └── lqr.py
├── simulations/
│   ├── pure_pursuit_sim.py
│   ├── stanley_sim.py
│   └── lqr_sim.py
├── vehicle/
│   ├── bicycle_model.py
│   └── bicycle_model_lqr.py
├── paths/
│   └── generate_path.py
├── utils/
│   ├── kalman_filter.py
│   ├── sensor_noise.py
│   ├── metrics.py
│   ├── plot_path_comparison.py
│   ├── plot_error_comparison.py
│   └── animation.py
├── results/figures/
├── main.py
└── report.pdf
```

---

## Vehicle Model

Kinematic bicycle model — assumes no tyre slip, valid at low to moderate speeds.

```
x_dot   = v · cos(ψ)
y_dot   = v · sin(ψ)
ψ_dot   = (v / L) · tan(δ)
```

**States:** position (x, y), heading ψ, velocity v  
**Inputs:** steering angle δ, acceleration a  
**Wheelbase:** L = 4 m

---

## Controllers

### Pure Pursuit
Geometric controller that steers toward a lookahead point at distance `L_d` from the rear axle.

```
δ = arctan(2 · L · sin(α) / L_d)
```

### Stanley
Error-based controller correcting both heading error and cross-track error at the front axle.

```
δ = k_ψ · ψ_e + arctan(k · e / v)
```

### LQR (Linear Quadratic Regulator)
Optimal controller minimising a quadratic cost function over tracking error and control effort.

```
J = ∫ (xᵀQx + uᵀRu) dt

State:  x = [e, ė, ψ_e, ψ̇_e, v_e]
Input:  u = [δ̇, a]
```

System linearised around the kinematic bicycle model. Gain matrix K computed by solving the Algebraic Riccati Equation. LQR uses curvature-adaptive speed profiling — target speed reduces on tight curves.

---

## Sensor Noise & Kalman Filter

Gaussian noise added to position and heading measurements to simulate realistic sensor conditions:

```
x_meas   = x   + N(0, 0.2)
y_meas   = y   + N(0, 0.2)
ψ_meas   = ψ   + N(0, 0.03)
```

A Kalman Filter estimates the vehicle state from noisy measurements using the bicycle model as the process model. The estimated state is passed to the controller instead of raw measurements.

---

## Results

### Performance Metrics (under sensor noise σ = 0.2 m)

| Path | Controller | RMS Error (m) | Max Error (m) | Mean Error (m) |
|------|-----------|--------------|--------------|----------------|
| Straight | Pure Pursuit | 0.143 | 0.595 | 0.125 |
| Straight | Stanley | 0.134 | 0.277 | 0.122 |
| Straight | LQR | 0.142 | 0.317 | 0.130 |
| Semi-circular | Pure Pursuit | 0.111 | 0.443 | 0.098 |
| Semi-circular | Stanley | 0.113 | 0.214 | 0.104 |
| Semi-circular | LQR | 0.124 | 0.367 | 0.109 |
| Lane Change | Pure Pursuit | 0.435 | 1.178 | 0.321 |
| Lane Change | Stanley | 0.304 | 0.968 | 0.212 |
| Lane Change | **LQR** | **0.300** | 0.992 | 0.264 |
| Sinusoidal | Pure Pursuit | 0.881 | 3.350 | 0.617 |
| Sinusoidal | Stanley | 1.102 | 4.102 | 0.635 |
| Sinusoidal | **LQR** | **0.671** | **1.813** | **0.486** |

Kalman Filter reduced LQR RMS tracking error by **~22%** on the sinusoidal trajectory (0.9 m → 0.7 m).

---

## Setup & Usage

### Requirements
```
numpy
scipy
matplotlib
```

Install:
```bash
pip install numpy scipy matplotlib
```

### Run
```bash
python main.py
```

This runs all three controllers on all four paths, plots trajectory and error comparisons, prints RMSE/max/mean metrics, and shows an animation of the LQR vehicle.

---

## Key Observations

- All three controllers performed similarly on straight and circular paths (RMSE < 0.15 m)
- Performance differences emerged on lane-change and sinusoidal paths where curvature varies rapidly
- **LQR** achieved lowest RMS and max error on the sinusoidal path
- **Stanley** achieved lowest RMS on lane-change path
- **Pure Pursuit** was simplest to implement and tune, with a single lookahead parameter
- Kalman Filter improved LQR performance most noticeably under sensor noise

---

## Future Work

- Extended Kalman Filter (EKF) with nonlinear bicycle model dynamics
- Dynamic bicycle model incorporating tyre forces and lateral slip
- Model Predictive Control (MPC)
- ROS2 and Gazebo integration

---

## Author

**Sujal Singh** — Personal Project  
B.Tech Mechanical Engineering, IIT Kharagpur  
[GitHub](https://github.com/SujalSingh555/autonomous-vehicle-path-tracking)
