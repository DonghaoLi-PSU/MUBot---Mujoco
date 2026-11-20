# MUBot Simulation

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![MuJoCo](https://img.shields.io/badge/MuJoCo-planned-lightgrey)

<!-- TODO: expand "MUBot" and link the paper / project page here. -->

Hydrodynamic simulation and gait-learning tools for **MUBot**, a multi-link swimming robot driven by magnetic-electric actuators and passive springs. The simulator models the actuators, the fluid forces on every link, and a set of drift metrics that score a gait. It is used to learn open-loop sinusoidal gaits with EM-PGPE.

The project is moving from Gazebo Classic + ROS 1 to ROS 2 Jazzy, and from there to MuJoCo.

| Stage | Stack | Status |
|-------|-------|--------|
| Legacy | Gazebo Classic + ROS 1 | Original implementation (2021) |
| **Current** | Gazebo Harmonic (gz-sim 8) + ROS 2 Jazzy | In use. The build and run instructions below target this stack |
| Next | MuJoCo + ROS 2 Jazzy | Planned, see [MuJoCo migration](#mujoco-migration) |

## Contents

- [What it simulates](#what-it-simulates)
- [Repository layout](#repository-layout)
- [Requirements and build](#requirements-and-build)
- [Running](#running)
- [Robot database](#robot-database)
- [MuJoCo migration](#mujoco-migration)
- [Credits](#credits)

## What it simulates

**Robot.** A neutrally buoyant, multi-link swimmer in a zero-gravity world. The reference model (`mubot.urdf`) has a head, four body segments and a tail joined by five revolute joints about the z axis. Link and joint counts are read from the robot's `Database/<id>/` folder, so morphologies can be swapped without recompiling.

**Actuators** (`actuator_plugin.cc`). Each joint is driven by a sinusoidal voltage and a back-EMF style torque law, plus an optional spring term:

```
V_i(t) = A_i · sin(2π (f·t + φ_i))                  clamped to ±15 V
τ_i    = scaling · [ ((V_i − ac_i·ω_i) / 90) · ac_i  −  sf_i·θ_i ]
```

`ω_i` and `θ_i` are joint velocity and position, `ac_i` and `sf_i` come from `joint_param.csv`, and the spring term is active only when `<explicit_spring>true</explicit_spring>` is set in the plugin block. The torque is then clamped to an inertia-based acceleration limit.

**Hydrodynamics** (`hydro_plugin.cc`). Evaluated for every link at every physics step, in the link frame:

- reactive (added-mass type) force and torque from link velocity and acceleration
- skin friction along the link axis and quadratic cross-flow drag, integrated along the segment
- a compensation term proportional to link mass times acceleration
- a pressure force at the tail tip
- clamping of the resulting wrench to keep the simulation stable

Per-link coefficients come from `hydro_param.csv`. Fluid and robot density are scaled by 1000 for numerical accuracy, and actuator torques are scaled to match.

**Scoring a gait.** Starting at t = 4 s and covering a whole number of oscillation cycles (ending before 6.1 s), the hydro plugin computes four drift metrics and writes them to `record/drift_metrics.csv`:

| Metric | Meaning |
|--------|---------|
| `CoGVel_x` | Mean mass-weighted CoG velocity along x |
| `abs_lateral` | Maximum lateral excursion of the CoG |
| `rms_lat` | RMS lateral excursion |
| `heading_drift` | Change in mean link yaw over the window |

```
reward = −CoGVel_x − max(abs_lateral − 0.1, 0) − 0.2 · max(heading_drift − 0.1, 0)
```

The body extends along +x from the head, so forward swimming is negative `CoGVel_x` and scores higher.

**Gait learning.** The open-loop controller has 2N parameters for N joints: the first amplitude, an amplitude and phase for each remaining joint, and one shared frequency. `EM_pgpe.py` implements EM-PGPE over those parameters, and `start_empgpe.py` runs the episodes and rollouts and keeps the history CSVs.

**Step-driven control.** For closed-loop or RL experiments the actuator plugin can run in external-control mode (`<external_control>true</external_control>`). It then exposes a gz-transport service, `/model/<name>/step`, and the simulation advances only as many physics steps as the caller requests, returning joint state, head state and CoG/tail state after each call. `mubot_step_client.py` wraps this service and `start_ppo.py` shows the rollout loop with a policy hook.

## Repository layout

The workspace is a standard colcon workspace at `~/MUBot---Mujoco` with three packages:

```
MUBot---Mujoco/src/
├── mubot_interfaces/                 # custom ROS 2 messages and services
├── mubot_control/                    # controller nodes
└── mubot_sim/
    ├── include/mubot_sim/mubot_common.hh   # CSV parsing, numerics, Database lookup
    ├── src/
    │   ├── actuator_plugin.cc        # voltage → joint torque, spring, step service
    │   └── hydro_plugin.cc           # per-link fluid forces, drift metrics
    ├── launch/
    │   ├── mubot.launch.py           # gz sim + ros_gz_bridge (world control)
    │   └── dump_mubot.launch.py      # gz sim only, no bridge
    ├── worlds/mubot.world            # zero gravity, 0.25 ms time step
    ├── scripts/
    │   ├── EM_pgpe.py                # EM-PGPE optimiser
    │   ├── start_empgpe.py           # training entry point
    │   ├── start_ppo.py              # lock-step rollouts with a policy hook
    │   ├── mubot_rob_env.py          # MubotEnv: owns the sim process and rollouts
    │   ├── mubot_step_client.py      # client for /model/<name>/step
    │   └── gazebo_connection.py      # pause / unpause / reset via ros_gz_bridge
    └── Database/<id>/                # per-robot model and parameters
```

## Requirements and build

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- `ros-jazzy-ros-gz` (provides `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`)
- Gazebo Python bindings for the step client: `python3-gz-transport13`, `python3-gz-msgs10`
- `numpy` and `pandas`

```bash
cd ~/MUBot---Mujoco/src
git clone <repo-url> .          # <!-- TODO: repo URL -->
cd ~/MUBot---Mujoco
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` is required: the launch file locates `Database/` in the source tree and refuses to run from a copied install.

## Running

**Single simulation** with the gait in `Database/<id>/control_signal.csv`:

```bash
ros2 launch mubot_sim mubot.launch.py robot_name:=mubot_1
```

| Argument | Default | Purpose |
|----------|---------|---------|
| `robot_name` | `mubot_1` | Must start with `mubot_`. Everything after the prefix selects the `Database/<id>/` folder |
| `database_dir` | `<package>/Database` | Root folder holding the `<id>/` subfolders |
| `world` | `worlds/mubot.world` | World file |
| `headless` | `false` | `true` runs the server only, with no GUI |
| `run` | `true` | `false` starts paused |

**Gait learning.** Run from a shell where ROS and the workspace are sourced:

```bash
cd ~/MUBot---Mujoco/src/mubot_sim/scripts
python3 start_empgpe.py --wk-id 1 --mode run                      # new run
python3 start_empgpe.py --wk-id 1 --mode continue                 # resume from the history CSVs
python3 start_empgpe.py --wk-id 1 --mode rerun --morph-id 17      # re-run a stored controller.csv
```

Add `--gui` to watch the rollouts. Each worker id gets its own Gazebo partition and ROS domain, so several workers can run on one machine at the same time.

## Robot database

Each robot lives in `Database/<id>/`. The plugins read these files at start-up and again on every world reset.

| File | Contents |
|------|----------|
| `mubot.urdf` | Robot model, including the plugin blocks |
| `joint_param.csv` | One row per joint: `name, sf_param, ac_param, stiff_ratio` |
| `hydro_param.csv` | One row per link, 13 columns: `name, M_total, M_S, M_0, M_l, cog, lt, h, IH0, IH1, IH2, IH3, IP` |
| `control_signal.csv` | One row of 2N numbers: `A_0, (A_i, φ_i) for i = 1…N−1, f` |
| `record/` | Outputs: `drift_metrics.csv`, `displace_record.csv`, `force_record.csv`, `actuator_output.csv`, `voltage_output.csv` |

Row counts and names must match the model's links and joints exactly. On any mismatch the plugin logs `plugin disabled` and the simulation does nothing.

## MuJoCo migration

**Why.** In the Gazebo setup every rollout depends on pausing, unpausing and resetting a separate simulator process through the ROS bridge, with file-based handshakes (`terminate.txt`, `reset_ack.txt`) between the plugin and the Python side. A training run is hundreds of rollouts, each 6.1 s of simulated time at 4 kHz (about 24,400 steps). Stepping MuJoCo in-process removes that overhead and makes parallel rollouts straightforward.

**Principles**

1. **Parity first.** The Gazebo Harmonic backend is the reference. The MuJoCo backend must reproduce it before any physics is changed.
2. **Same data contract.** The `Database/<id>/` files and `record/` outputs keep their formats, so `start_empgpe.py` and `EM_pgpe.py` run unchanged on either backend. `MubotEnv` is the seam: `reset`, `advance`, `finish` and `read_metrics` are what a backend has to provide.
3. **ROS 2 stays the interface.** The MuJoCo model runs inside a ROS 2 Jazzy node, with services defined in `mubot_interfaces`.

**Design notes**

- *Base joints.* Gravity is off and all forces act in the plane, so the base can be x, y and yaw joints instead of a free joint. Check that this matches the floating base in Gazebo.
- *Plugins become callbacks.* Actuator torque is applied from a control callback. The hydro wrench is applied through `xfrc_applied`. Gazebo applies the force at an offset along the link axis (`-cog`), which has to be converted into an equivalent force and torque about the body's center of mass.
- *Accelerations.* The reactive terms use link acceleration, and a callback runs before the step, so it sees the previous step's value. Options are last-step acceleration, a finite difference of velocity, or folding added mass into the body inertia. The Gazebo plugin already zeroes accelerations on the first step after a reset, and the port has to handle the same case.
- *Fluid model.* MuJoCo has a built-in fluid model (`density` and `viscosity`). Port the existing per-link model first and evaluate the built-in one afterwards.
- *Numerics.* Keep the 0.25 ms time step and the ×1000 density and torque scaling until parity is shown.

**Roadmap**

- [x] Gazebo Harmonic plugins (actuator, hydro) and ROS 2 Jazzy launch files
- [x] Lock-step step service and `MubotEnv` rollout API
- [ ] MJCF model generated from `Database/<id>/mubot.urdf`
- [ ] Actuator model as a MuJoCo control callback
- [ ] Hydro model port, including tail pressure and wrench clamping
- [ ] Drift metrics and record files in the existing CSV formats
- [ ] ROS 2 node for the MuJoCo backend, with `reset` and `step` services in `mubot_interfaces`
- [ ] Backend switch in `MubotEnv` and a `backend:=gazebo|mujoco` launch argument
- [ ] Parity test against Gazebo (see below)
- [ ] Parallel in-process rollouts for EM-PGPE

**Parity test.** For a set of reference gaits, run the same `control_signal.csv` on both backends and compare the four drift metrics and the CoG trajectory. The tolerances are to be agreed before the MuJoCo backend is accepted. EM-PGPE reward curves on both backends should also be comparable.

## Credits

Original Gazebo Classic plugins and EM-PGPE tooling by Donghao Li (2021). Ported to Gazebo Harmonic and ROS 2 Jazzy.

<!-- TODO: add LICENSE and reference it here. -->
<!-- TODO: add citation (BibTeX) once the paper is public. -->
