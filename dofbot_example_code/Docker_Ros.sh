#!/bin/bash

# Wait for the Docker service to start
while true; do
    if systemctl is-active --quiet docker; then
        echo "Docker service has been started"
        break
    else
        echo "The Docker service has not started, waiting..."
        sleep 1
    fi
done

# Docker start
xhost +

docker run -it --rm \
--privileged=true \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /home/pi/Temp:/root/Temp \
--device=/dev/video0 \
--device=/dev/i2c-1 \
yahboomtechnology/ros-melodic:4.4 /bin/bash