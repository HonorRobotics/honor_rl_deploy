# Humanoid Control (C++)

<div align="center">

**RL policy deployment for the VitaBoy humanoid, one controller, one FSM, one set of ROS 2 topics driving MuJoCo and the real robot identically.**

[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![ONNX Runtime](https://img.shields.io/badge/inference-ONNX%20Runtime-005CED?logo=onnx&logoColor=white)](https://onnxruntime.ai/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)

[English](readme.md) | [简体中文](readme_zh.md)

</div>

## ✨ Overview

The controller runs a finite state machine, reads joystick commands and robot state over ROS 2, runs the ONNX policy, and publishes joint commands. The same code drives the [`honor_mujoco`](https://github.com/HonorRobotics/honor_mujoco) simulation and the real robot over the *same* ROS 2 topics.

![Overview diagram](docs/overview.svg)

## 🧩 FSM States


| State        | Purpose                                                    |
| ------------ | ---------------------------------------------------------- |
| `Passive`    | Zero motor command (boot / safest state)                   |
| `Damper`     | Pure joint damping (kp = 0, kd from config)                |
| `Recovery`   | Cosine-interpolate from the current pose to the stand pose |
| `Locomotion` | Velocity-tracking locomotion driven by an ONNX policy      |
| `Tracking`   | Whole-body motion tracking driven by ONNX policies         |

## 📦 Setup

### Requirements

- Linux (Ubuntu 22.04)
- [uv](https://docs.astral.sh/uv/getting-started/installation/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Clone the project

Clone this repository and [`honor_robot_sdk`](https://github.com/HonorRobotics/honor_robot_sdk) side by side:

```bash
git clone https://github.com/HonorRobotics/honor_rl_deploy.git

git clone https://github.com/HonorRobotics/honor_robot_sdk.git
```

### Install dependencies

```bash
cd honor_rl_deploy

./scripts/install.sh
```

### Check the environment

Verifies every build dependency

```bash
./scripts/check_env.sh
```

## 🚀 Usage

### Sim-robot

#### Step 1: Install the SDK

First clone the [`honor_mujoco`](https://github.com/HonorRobotics/honor_mujoco) repo and install its dependencies and the SDK.

Open a new terminal:

```bash
cd honor_mujoco

source ./scripts/build_sdk_msgs.sh --honor-sdk /your_path/to_sdk
```

#### Step 2: Start MuJoCo

```bash
./scripts/run_mujoco.sh
```

#### Step 3: Start the controller

In a new terminal:

```bash
cd honor_rl_deploy

./scripts/run_motion_intelligence.sh \
    --fastdds false \
    --is-sim true \
    --joy-topic /joy \
    --joy-type keyboard
```

- ✅ No F710 gamepad: keyboard control is used by default — see [keyboard controls](#keyboard-controls)
- ✅ Have an F710 gamepad: pass `--joy-type game_controller` instead — see [Joystick controls](#joystick-controls)

### Real-robot

#### Step 1: Power on

Power on the vita_boy robot and connect it through a network cable.

After the VitaBoy robot boots, the pre-flashed `motion_intelligence` controller is enabled by default, with joints in **zero-torque** (Passive).

Note: make sure no network cable other than the one to the robot is plugged into your computer.

#### Step 2: Enter debug mode

Press `S1 + S2` on the remote to enter debug mode.

Joints switch to **Damper**, and only then can this repo's code control the robot.

#### Step 3: Run the controller

##### Scenario 1: tune new controller with an external host

Run on your local computer's terminal:

**Step 1: Install the SDK**

In a new terminal:

```bash
cd honor_rl_deploy

source ./scripts/build_sdk_msgs.sh --honor-sdk /your_path/to_sdk
```

**Step 2: Pre-deployment check**

Set your computer's IP to `192.168.42.xxx` (e.g. `192.168.42.200`), then run the check:

```bash
./scripts/check_robot.sh \
    --joy-check false \
    --joy-topic /joy
```

It checks the network, SSH login + `motion_intelligence.service` state, and the ROS 2 layer. Make sure every check passes before continuing with either scenario below.

**Step 3: Start the controller**

```bash
./scripts/run_motion_intelligence.sh \
    --fastdds true \
    --is-sim false \
    --joy-topic /joy \
    --joy-type keyboard
```

Then control the robot with the [keyboard controls](#keyboard-controls) or [Joystick controls](#joystick-controls).

- ✅ No F710 gamepad: keyboard control is used by default — see [keyboard controls](#keyboard-controls)
- ✅ Have an F710 gamepad: pass `--joy-type game_controller` instead — see [Joystick controls](#joystick-controls)

##### Scenario 2: tune new controller on the onboard computer

**Step 1: Copy the local project**

On your local computer, zip up the cloned code, then copy it to the onboard computer with the commands below.

Note: if the code has already been built, delete the build artifacts first — `honor_rl_deploy/source/build`, `honor_robot_sdk/common/build`, `honor_robot_sdk/common/build_dist`.

```bash
scp /path/to/your/honor_rl_deploy.zip hihonor@192.168.42.201:/data/your_path/

scp /path/to/your/honor_robot_sdk.zip hihonor@192.168.42.201:/data/your_path/
```

**Step 2: SSH into the onboard computer**

```bash
ssh hihonor@192.168.42.201

password hihonor
```

**Step 3: Unzip the copied project**

Note: steps 3, 4, 5, and 6 are all run in the onboard computer's SSH session.

```bash
unzip /path/to/your/honor_rl_deploy.zip

unzip /path/to/your/honor_robot_sdk.zip
```

**Step 4: Install the SDK**

```bash
cd /path/to/your/honor_rl_deploy

source ./scripts/build_sdk_msgs.sh --honor-sdk /your_path/to_sdk
```

**Step 5: Pre-deployment check**

```bash
./scripts/check_robot.sh \
    --joy-check true \
    --joy-topic /xlab/hr/joy_state_debug
```

check_robot.sh checks the network, SSH login + `motion_intelligence.service` state, and the ROS 2 layer. Make sure every check passes before continuing with either scenario below.

**Step 6: Build and run the controller**

```bash
./scripts/run_motion_intelligence.sh \
    --fastdds true \
    --is-sim false \
    --joy-topic /xlab/hr/joy_state_debug \
    --joy-type robot_remote_control
```

Then control the robot with the [Remote controls](#remote-controls).

### Key bindings

#### Joystick controls

Set `joy_type: "game_controller"` in [`base.yaml`](source/config/vita_boy/V1/base.yaml), or pass `--joy-type game_controller` to `run_motion_intelligence.sh`.


| Combo                            | Target state                       |
| -------------------------------- | ---------------------------------- |
| `LB` + `A`                       | Passive                            |
| `LB` + `B`                       | Damper                             |
| `LB` + `X`                       | Recovery                           |
| `LB` + `Y`                       | Locomotion                         |
| `RB` + `X` (while in Locomotion) | Tracking                           |
| `LB` + `RB`                      | Damper (one-button emergency stop) |

Left stick (while in Locomotion): ↕ forward / backward (`vx`), ↔ strafe left / right (`vy`).
Right stick (while in Locomotion): ↔ turn left / right (`vz`).

#### Remote controls

Set `joy_type: "robot_remote_control"` in [`base.yaml`](source/config/vita_boy/V1/base.yaml), or pass `--joy-type robot_remote_control` to `run_motion_intelligence.sh`.


| Remote combo                      | Target state                       |
| --------------------------------- | ---------------------------------- |
| `S1` + `R1`                       | Passive                            |
| `S1` + `R2`                       | Recovery                           |
| `S1` + `M6`                       | Damper                             |
| `S1` + `R3`                       | Locomotion                         |
| `M2` + `R2` (while in Locomotion) | Tracking                           |
| `S1` + `M2`                       | Damper (one-button emergency stop) |
| `S1` + `S2`                       | Enter debug mode                   |

Left stick (while in Locomotion): ↕ forward / backward (`vx`), ↔ strafe left / right (`vy`).
Right stick (while in Locomotion): ↔ turn left / right (`vz`).

#### keyboard controls

Set `joy_type: "keyboard"` in [`base.yaml`](source/config/vita_boy/V1/base.yaml), or pass `--joy-type keyboard` to `run_motion_intelligence.sh`.


| Key                             | Target state         |
| ------------------------------- | -------------------- |
| `P`                             | Passive              |
| `Space`                         | Damper               |
| `R`                             | Recovery             |
| `L`                             | Locomotion           |
| `M` (while in Locomotion)       | Tracking             |

Movement (while in Locomotion): `W`/`S` forward / backward (`vx`); `A`/`D` strafe left / right (`vy`); `Q`/`E` turn left / right (`vz`).

💡 **Tip:**

1. For sim2sim, the table above requires the terminal running `run_motion_intelligence.sh` to be focused with a mouse click; MuJoCo-side keys require the MuJoCo window to be focused. Once in Locomotion, we suggest clicking the MuJoCo window and pressing `9` to set the robot down, then clicking back on the `run_motion_intelligence.sh` terminal to use the table above.
2. Suggested first bring-up order — **Passive → Recovery → Locomotion → Tracking → Locomotion → Damper**.

## 🛠️ Development

```text
<workspace>/
└── honor_rl_deploy/
    ├── source/
    │   ├── src/, include/                 # controller, state machine, estimator, command, hardware interface
    │   ├── config/vita_boy/V1/            # base.yaml, command.yaml and the per-state policy configs
    │   └── thirdparty/                    # onnxruntime, cnpy, argparse
    └── scripts/
        ├── install.sh                     # checks uv + ROS, creates .venv, installs deps
        ├── build_sdk_msgs.sh              # build honor_robot_sdk message packages
        ├── check_env.sh                   # verifies the full C++ build environment
        ├── check_robot.sh                 # pre-deployment check: network + SSH + ROS 2 topics
        ├── run_motion_intelligence.sh     # builds the project + controller
        └── fastdds_profile.xml.template   # Fast DDS interface whitelist config
```
