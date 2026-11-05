# Multi-Drone Formation Flight Simulation (MuJoCo + Crazyflie 2)

Simulation of small quadrotor swarms flying in formation, written as a single Google Colab notebook (`Code.ipynb`).

- **Part 1 (4 drones):** single-drone PID hover, a 4-drone line, a mock `PlutoInterface` (RC channels to actuators), line and circle mission planning, and a cascaded position/velocity PID controller.
- **Part 2 (10 drones):** 10-drone line and circle formations with a vertical "Mexican wave", Hungarian slot assignment, minimum-jerk trajectories, altitude-layer collision avoidance, a geometric attitude controller, and a pre-flight and post-flight safety evaluation.

---

## Simulator

| Item | Value |
|---|---|
| Physics engine | **MuJoCo** (`mujoco`) |
| MuJoCo version used | **3.17.0** (notebook pins `mujoco>=3.2,<4`) |
| Vehicle model | **Bitcraze Crazyflie 2** (`bitcraze_crazyflie_2/cf2.xml`) from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) |
| Menagerie version | Not pinned. The notebook does a shallow sparse clone of the default branch (`--depth 1`), so it gets whatever is latest at run time |
| Swarm model | The single `cf2` body, actuators and sensors are duplicated N times in code and written to `four_crazyflies.xml` / `swarm_10_crazyflies.xml` |
| Physics rate | 500 Hz (`timestep = 0.002 s`), RK4 integrator |
| Outer control rate | 25 Hz (Part 1), 50 Hz (Part 2) |
| Fallback backend | Pure NumPy rigid-body quadrotor (Part 2 only), used when `BACKEND="numpy"` or if MuJoCo fails to load |
| Rendering | Matplotlib animation (Part 2) and MuJoCo offscreen EGL renderer (Part 1, `MUJOCO_GL=egl`) |
| Runtime | Google Colab (paths use `/content`) |

### Library versions (from the notebook's saved Colab run)

| Package | Version |
|---|---|
| Python | 3.13 |
| mujoco | 3.17.0 |
| numpy | 2.4.6 |
| scipy | 1.16.3 |
| pandas | 2.2.3 |
| matplotlib | 3.10.0 |
| opencv-contrib-python | 4.14.0.94 |
| pygame | 2.6.1 |
| PyYAML | 6.0.3 |
| pybullet | 3.2.7 |
| gym-pybullet-drones | 2.2.0 |
| imageio, imageio-ffmpeg | unpinned (latest at install time) |

Notes:
- `pybullet` and `gym-pybullet-drones` are installed in the first cells but **the flight code does not use them**; everything runs on MuJoCo. They can be removed.
- Part 2 starts with its own `%pip install "mujoco>=3.2,<4" imageio imageio-ffmpeg scipy pandas matplotlib`.
- Library versions above come from the Colab session that produced the saved outputs; Colab's preinstalled versions change over time, so for exact reproducibility pin them in a `requirements.txt`.

### Crazyflie 2 parameters (as measured by the notebook)

| Parameter | Value |
|---|---|
| Mass | 0.027 kg |
| Maximum thrust | 0.35 N (thrust-to-weight about 1.32) |
| Hover thrust | about 0.265 N (about 76 % of max) |
| Actuators per drone | `body_thrust`, `x_moment`, `y_moment`, `z_moment` |

---

## Arena and safety constants

| Constant | Value |
|---|---|
| X range | -1.0 to 1.0 m |
| Y range | -0.6 to 0.6 m |
| Z range (airborne) | 0.6 to 1.4 m |
| Minimum 3-D separation | 0.35 m |
| Maximum tilt | 15 degrees |
| Wave | 0.2 m amplitude, 6 s period, 3 cycles |
| Base / start / landing height | 1.0 / 0.1 / 0.1 m |

---

## Notebook layout

| Cells | Content |
|---|---|
| 1-7 | Package installs (pybullet, gym-pybullet-drones, numpy/scipy, OpenCV, pandas, PyYAML, matplotlib/pygame) |
| 8-11 | Menagerie download, MuJoCo model load, actuator list, Crazyflie mass/inertia/thrust |
| 12-15 | Single-drone altitude PID hover test, plots, video, downloads |
| 16-18 | Re-install and reload of MuJoCo and the Crazyflie model |
| 19-23 | 4-drone MJCF generation, model load, first `PlutoInterface`, initial separation check |
| 24-31 | Altitude PID for 4 drones, simulation, plots, video |
| 32-46 | Line/circle formation generators, Hungarian assignment, quintic blending, wave, mission planner, validators, N=10 feasibility check, plan export |
| 47-51 | Wave simulation in MuJoCo, actual vs target comparison, video |
| 52-65 | Updated `PlutoInterface` (RC channels), cascaded position/velocity PID, line and circle missions, metrics, plots, videos, logs |
| 66-68 | Geofence and separation diagnostics for the circle mission |
| 69-83 | **Part 2:** 10-drone planner, MuJoCo and NumPy backends, controller, evaluation, CSV and video export |


---

## Design notes

1. **A straight 10-drone line does not fit.** At 0.35 m separation it needs 3.15 m along X, but the arena is 2 m wide. The default `LINE_STYLE = "zigzag"` (a staggered two-rank line, minimum spacing 0.377 m) fits. Set `LINE_STYLE = "straight"` to see the rejection.
2. **The 10-drone circle is tight.** The minimum radius is 0.566 m; the notebook uses 0.59 m, which leaves 14.6 mm of margin over 0.35 m.
3. **Backend selection.** `BACKEND = "auto"` tries MuJoCo first and falls back to NumPy. Use `"numpy"` or `"mujoco"` to force one.

## Known issue

With the MuJoCo backend, the saved 10-drone run **fails its checks** (separation, geofence, tilt, torque saturation). The start-up calibration reports a maximum angular acceleration of roughly 0 rad/s² for the Menagerie moment actuators, so the attitude loop (clamped to a minimum of 8 rad/s) cannot be tracked and the drones flip. The NumPy backend, with realistic torque authority, passes every check for both formations (3-D tracking RMSE about 0.2 mm, minimum separation 0.377 m line / 0.359 m circle, maximum tilt under 1 degree). For passing results today, set `BACKEND = "numpy"` in the "RUN BOTH MISSIONS" cell.

---

## How to run

1. Open `Code.ipynb` in Google Colab.
2. Run the cells top to bottom (Part 1 first, then Part 2).
3. Outputs are written to `/content`: CSV logs (`line_10drones_log.csv`, `circle_10drones_log.csv`, and the Part 1 logs), PNG figures and MP4 videos.

## References

- E. Todorov, T. Erez, Y. Tassa, "MuJoCo: A physics engine for model-based control," IROS 2012.
- MuJoCo Menagerie, Bitcraze Crazyflie 2 model: github.com/google-deepmind/mujoco_menagerie
- D. Mellinger, V. Kumar, "Minimum snap trajectory generation and control for quadrotors," ICRA 2011.
- T. Lee, M. Leok, N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)," CDC 2010.
- M. Turpin, N. Michael, V. Kumar, "CAPT: Concurrent assignment and planning of trajectories for multiple robots," IJRR 2014.
- H. W. Kuhn, "The Hungarian method for the assignment problem," 1955.

Please check the bibliographic details against the original sources before submitting.
