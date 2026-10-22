FROM debian:bookworm-slim

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        git \
        make \
        ninja-build \
        python3 \
        python3-pip \
        gcc-arm-none-eabi \
        binutils-arm-none-eabi \
        libnewlib-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["bash"]
