# Reproducible Yertle Isaac Lab training image.
# Versions match the running yertle-isaac container: Sim 5.1.0, Lab 2.3.2.
ARG ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0
FROM ${ISAAC_SIM_IMAGE}

ARG ISAACLAB_VERSION=2.3.2
USER root
ENV TERM=xterm

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
        ca-certificates git build-essential cmake \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch "v${ISAACLAB_VERSION}" \
        https://github.com/isaac-sim/IsaacLab.git /workspace/isaaclab \
    && ln -s /isaac-sim /workspace/isaaclab/_isaac_sim \
    && /workspace/isaaclab/_isaac_sim/python.sh -m pip install --no-cache-dir "setuptools<81" \
    && /workspace/isaaclab/_isaac_sim/python.sh -m pip install --no-cache-dir --no-build-isolation \
        -e /workspace/isaaclab/source/isaaclab \
    && /workspace/isaaclab/isaaclab.sh --install rsl_rl \
    && /workspace/isaaclab/isaaclab.sh -p -c "import isaaclab; print(isaaclab.__version__)"

COPY docker/container-entrypoint.sh /usr/local/bin/yertle-container-entrypoint
COPY docker/source-init.sh /usr/local/bin/yertle-source-init
COPY docker/train.sh /usr/local/bin/yertle-train
COPY docker/play.sh /usr/local/bin/yertle-play
COPY docker/sync-source.sh /usr/local/bin/yertle-source-sync
RUN chmod 0755 /usr/local/bin/yertle-container-entrypoint \
    /usr/local/bin/yertle-source-init \
    /usr/local/bin/yertle-train \
    /usr/local/bin/yertle-play \
    /usr/local/bin/yertle-source-sync

ENV ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    TERM=xterm \
    ISAACLAB_PATH=/workspace/isaaclab

ENTRYPOINT ["/usr/local/bin/yertle-container-entrypoint"]
CMD ["bash"]
