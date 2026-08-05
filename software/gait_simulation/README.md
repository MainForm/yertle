# Yertle Gait Simulation

This project simulates the Yertle robot's gait using keyboard controls. It
supports forward and backward movement and left and right turns, with each foot
following a trajectory defined by a function.

## Prerequisites

- Docker Engine
- Docker Compose v2 (`docker compose`)
- An X11 display server for graphical applications such as the PyBullet GUI

## Start the Development Container

Run the following commands from this directory:

```bash
cd software/gait_simulation
docker compose --profile dev up --build -d
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

The example opens the PyBullet GUI, loads a ground plane and a sample robot, and
runs until you press `Ctrl+C`.

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

These packages are installed directly in the Dockerfile instead of through the
repository-level `requirements.txt` file.

## Stop the Development Container

Exit the container shell, then stop and remove the Compose resources:

```bash
docker compose --profile dev down
```
