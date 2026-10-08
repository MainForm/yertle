# Yertle Gait Simulation

This project simulates the Yertle robot's gait using keyboard controls. It
loads the Yertle URDF in PyBullet so that gait development and testing can be
done without the physical robot. **The forward gait is now complete.**
Backward walking and left/right turns are not implemented yet.

## Forward Gait

The forward gait alternates two diagonal leg pairs: left front + right rear
(LF + RB) and right front + left rear (RF + LB). The pairs move half a cycle
apart. Each foot follows a sinusoidal forward/backward trajectory and lifts
during the swing phase, while its lift height stays at zero during the stance
phase. Inverse kinematics converts these foot targets into thigh and shin
joint angles.

## Keyboard Controls

After starting the `gait_simulation` service, click the PyBullet simulation
window to give it keyboard focus.

| Input | Action |
| --- | --- |
| Hold **↑ (Up arrow)** | Move forward using the completed forward gait. |
| Release **↑ (Up arrow)** | Stop walking and return the legs to their standing pose. |
| `Ctrl+C` in the terminal | Stop the simulation. |

## Prerequisites

- Docker Engine
- Docker Compose v2 (`docker compose`)
- An X11 display server for graphical applications such as the PyBullet GUI

## Docker Compose Services

The Compose configuration provides the following services:

| Service | Description |
| --- | --- |
| `gait_simulation` | Runs `main.py` and opens the interactive PyBullet gait simulation. |
| `show-forward-gait-plot` | Runs `gait_visualization/foward_gait.py` and displays the forward-gait plots and animation with Matplotlib. |
| `yertle-dev` | Starts an interactive Bash shell for development and debugging. This service is available through the `dev` profile. |

Run the following commands from `software/gait_simulation`. To start the
PyBullet gait simulation:

```bash
docker compose up --build gait_simulation
```

Display the forward-gait plots and animation:

```bash
docker compose up --build show-forward-gait-plot
```

Both services run in the foreground and stop when their GUI window is closed
or when you press `Ctrl+C`.

## Start the Development Container

Run the following commands from this directory:

```bash
cd software/gait_simulation
docker compose --profile dev up --build -d yertle-dev
```

The first command selects this directory as the Docker Compose project. The
second command builds the image and starts the `yertle-dev` service in the
background.

Open an interactive shell in the running container:

```bash
docker compose exec yertle-dev bash
```

The local directory is mounted at `/app`, so changes made on the host are
immediately available inside the container.

Run the example PyBullet simulation from the container shell:

```bash
python main.py
```

The simulation opens the PyBullet GUI and loads a ground plane and the Yertle
robot. Hold **↑ (Up arrow)** to move forward and release it to stop walking.
The simulation runs until you close the GUI window or press `Ctrl+C`.

## GUI Support on Linux

The Compose configuration forwards the host's `DISPLAY` variable and mounts the
X11 socket into the container. If the container cannot open a GUI window, allow
your local user to access the X server before starting it:

```bash
xhost +SI:localuser:$(whoami)
```

Revoke the permission when you are finished:

```bash
xhost -SI:localuser:$(whoami)
```

## Installed Python Libraries

- `numpy`: Numerical, matrix, and vector computations
- `pybullet`: Robot physics and gait simulation
- `ikpy`: Inverse kinematics calculations for robot joints
- `keyboard`: Keyboard input for interactive simulation control
- `matplotlib`: Plotting and animating gait trajectories

These packages are installed directly in the Dockerfile instead of through the
repository-level `requirements.txt` file.

## Stop the Compose Services

Stop and remove the Compose resources:

```bash
docker compose --profile dev down
```
