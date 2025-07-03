# --- Stage 1: Base ROS 2 Humble Image ---
# Use the official ROS 2 Humble Desktop image from OSRF. 'desktop' includes GUI tools.
FROM osrf/ros:humble-desktop

# Set the shell to use for subsequent RUN commands
SHELL ["/bin/bash", "-c"]

# --- System-Level Dependencies ---
# Set DEBIAN_FRONTEND to noninteractive to prevent prompts during installation
ARG DEBIAN_FRONTEND=noninteractive

# Update package lists and install necessary system libraries.
# python3-pip: for installing Python packages
# git: good practice to have inside containers
# python3-tk: ABSOLUTELY ESSENTIAL for your Tkinter UI to run
# libgl1-mesa-glx: Often required by OpenCV for GUI operations (cv2.imshow, etc.)
# In Dockerfile - This is the final lean version
RUN apt-get update && apt-get install -y --allow-insecure-repositories --allow-unauthenticated \
    python3-pip \
    git \
    python3-tk \
    libgl1-mesa-glx \
    xterm \
    libzbar0 \
    && rm -rf /var/lib/apt/lists/*

# In Dockerfile, after the RUN apt-get... line
ENV QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/

# --- Python Dependencies ---
# Create a directory for our Python requirements file
WORKDIR /tmp
# Copy JUST the requirements file first. Docker caches this layer. If requirements.txt
# doesn't change, Docker will reuse the cached layer on subsequent builds, speeding them up.
COPY src/smart_labeling_system_pkg/requirements.txt .
# Install all Python packages
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --ignore-installed -r requirements.txt

# --- ROS 2 Workspace Setup ---
# Create and set the working directory for our ROS 2 workspace inside the container
WORKDIR /ros2_ws

# Copy your entire 'src' folder (which contains your ROS packages) into the container
COPY src/ ./src/

# --- Build the ROS 2 Workspace ---
# Source the main ROS 2 environment, then run colcon build to compile your packages
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select smart_labeling_interfaces smart_labeling_system_pkg

# --- Entrypoint Configuration ---
# Copy the entrypoint script into the container's root directory
COPY src/smart_labeling_system_pkg/docker/ros_entrypoint.sh /
# Make the entrypoint script executable
RUN chmod +x /ros_entrypoint.sh
# Set this script as the entrypoint for the container. It will run every time the container starts.
ENTRYPOINT ["/ros_entrypoint.sh"]

# --- Default Command ---
# This is the command that will be executed by the entrypoint script by default.
# It launches your entire system.
CMD ["ros2", "launch", "smart_labeling_system_pkg", "system_launch.py"]
