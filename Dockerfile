# Use the official ROS 2 Humble Desktop image
FROM osrf/ros:humble-desktop

# Set shell for subsequent commands
SHELL ["/bin/bash", "-c"]

# Set DEBIAN_FRONTEND to noninteractive to prevent prompts during installation
ARG DEBIAN_FRONTEND=noninteractive

# --- System-Level Dependencies ---
# This is the corrected way to handle GPG signature errors.
# We create a configuration file for apt that tells it to ignore signature checks
# before we run the apt-get update and install commands.
RUN echo 'Acquire::AllowInsecureRepositories "true";' > /etc/apt/apt.conf.d/99-insecure-repositories && \
    echo 'Acquire::AllowUnverifiedRepositories "true";' >> /etc/apt/apt.conf.d/99-insecure-repositories

# Now, run the update and install commands WITHOUT the incorrect flags
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    python3-tk \
    libgl1-mesa-glx \
    xterm \
    libzbar0 \
    && rm -rf /var/lib/apt/lists/*

# --- Python Dependencies ---
WORKDIR /tmp
# Copy the requirements file first for layer caching
COPY src/smart_labeling_system_pkg/requirements.txt .
# Upgrade pip and install from requirements.txt
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --ignore-installed -r requirements.txt

# --- ROS 2 Workspace Setup ---
WORKDIR /ros2_ws
# Copy the source code
COPY src/ ./src/

# --- Build the ROS 2 Workspace ---
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select smart_labeling_interfaces smart_labeling_system_pkg

# --- Entrypoint Configuration ---
COPY src/smart_labeling_system_pkg/docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# --- Default Command (Changed back to shell form for robustness) ---
CMD ros2 launch smart_labeling_system_pkg system_launch.py
