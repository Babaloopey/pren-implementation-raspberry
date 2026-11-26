# PC simulator image: ROS 2 Jazzy (Ubuntu 24.04, same as the Raspberry Pi) + everything the
# robot nodes need. The source code is mounted at runtime (docker-compose.yml), not baked in.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-nav-msgs ros-jazzy-rosidl-default-generators \
        python3-pip python3-opencv python3-numpy python3-scipy python3-colcon-common-extensions \
        libportaudio2 python3-gpiozero \
    && rm -rf /var/lib/apt/lists/*

# Same pip packages as on the Pi (README). numpy stays < 2 because cv_bridge from apt is built against numpy 1.
RUN pip3 install "numpy<2" onnxruntime mediapipe sounddevice scikit-learn tqdm requests \
    && pip3 install --no-deps openwakeword \
    && python3 -c "import openwakeword.utils as u; u.download_models()" \
    && python3 -c "import numpy, cv2, mediapipe, onnxruntime, openwakeword; print('numpy', numpy.__version__, 'cv2', cv2.__version__)"

# MediaPipe (gesture_detector) loads OpenGL ES / EGL even on the CPU -> libGLESv2.so.2, libEGL.so.1
RUN apt-get update && apt-get install -y --no-install-recommends libgles2 libegl1 libgl1 \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

WORKDIR /ws
ENV ROS_DOMAIN_ID=17 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robot_sim", "sim.launch.py"]
