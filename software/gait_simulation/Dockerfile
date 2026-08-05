FROM ubuntu:24.04

# Disable interactive prompts during package installation and stream Python logs immediately.
# Install Python packages in the virtual environment at /opt/venv.
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    VIRTUAL_ENV=/opt/venv

ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

# Install Python and the system libraries required to run the PyBullet GUI.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        libgl1 \
        libglu1-mesa \
        libx11-6 \
        libxcursor1 \
        libxext6 \
        libxi6 \
        libxinerama1 \
        libxrandr2 \
        libxrender1 \
        libxss1 \
        python3 \
        python3-dev \
        python3-pip \
        python3-tk \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Create the project virtual environment and upgrade pip.
# Install the required libraries directly instead of using requirements.txt.
# - numpy: Matrix, vector, and numerical computations
# - pybullet: Robot physics and gait simulation
# - ikpy: Inverse kinematics calculations for robot joints
# - keyboard: Keyboard controls for the simulation
RUN python3 -m venv "${VIRTUAL_ENV}" \
    && python -m pip install --upgrade pip \
    && python -m pip install \
        "numpy>=1.24" \
        "pybullet>=3.2" \
        "ikpy>=3.3" \
        "keyboard>=0.13"

CMD ["/bin/bash"]
