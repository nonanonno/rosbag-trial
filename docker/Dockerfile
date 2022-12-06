ARG ROS_DISTRO
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG ROS_DISTRO

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-pip \
    && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install packages for Dlang
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libxml2 \
    clang \
    lld \
    lldb \
    curl \
    wget \
    xz-utils \
    ca-certificates \
    zlib1g-dev \
    libssl-dev \
    && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


ARG DC
RUN mkdir -p /opt/dlang && curl -fsS https://dlang.org/install.sh | bash -s -- $DC -p /opt/dlang
RUN find /opt/dlang -name activate | xargs echo source | tee -a /etc/bash.bashrc \
    && find /opt/dlang -maxdepth 1 -mindepth 1 -type d | xargs chmod 755

RUN ln -sf lld /usr/bin/ld

# Install pip packages
RUN pip install \
    jupyter \
    black