# syntax=docker/dockerfile:1
FROM python:3.9-slim AS builder

# Make a working directory
WORKDIR /ardupilot

################################################################################
### Install minimal build tools and remove cache. Don't do any update

RUN apt-get update && apt-get install --no-install-recommends -y \
    g++ \
    git \
    libxml2-dev \
    libxslt-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN python3 -m pip install --user --no-deps --no-cache-dir future empy==3.3.4 pexpect ptyprocess pymavlink

# Clone simplified. Don't clone all the tree and nuttx stuff
# put pymavlink to latest to work with python3 and recent gcc
RUN git clone https://github.com/poisonwhipsnake/ardupilot.git --depth 1 --no-single-branch /ardupilot \
    && cd /ardupilot \
    && git checkout main \
    && git submodule update --init --recursive --depth 1 modules/mavlink \
    && git submodule update --init --recursive --depth 1 modules/DroneCAN \
    && git submodule update --init --recursive --depth 1 modules/waf

# Build binary
RUN ./waf configure --board sitl --no-submodule-update \
    && ./waf rover

# Second stage build
FROM python:3.9-slim

WORKDIR /ardupilot

# copy only the dependencies installation from the 1st stage image
COPY --from=builder /root/.local /root/.local

# Copy binary and defaut param file from previous image
COPY --from=builder /ardupilot/build/sitl/bin/ardurover .
COPY --from=builder /ardupilot/Tools/autotest/default_params/rover.parm .
COPY sitl_launch.bash .

ENTRYPOINT ["./sitl_launch.bash"]
CMD ["bash"]