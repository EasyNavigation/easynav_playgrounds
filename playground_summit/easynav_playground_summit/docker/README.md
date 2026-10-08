# EasyNav ROS 2 rolling - Docker

This repository contains the 'Dockerfile' to use SummitXL playground, the image builds a ROS2 rolling environment and the EasyNav workspace ready to use. Also contains a launcher for a easy use of the docker.

## Instalation

```bash
cd <easynav_playgrounds>/playground_summit/easynav_playground_summit/docker/

docker build -t easynav_playground:summit_rolling .
```

## Usage

### Option A: using the script `launch.sh`
 
The first time Grant execute permissions to the script:
```bash
cd <easynav_playgrounds>/playground_summit/easynav_playground_summit/docker/

chmod +x launch.sh
```
 
Execute:

We can launch the docker with different options. --auto or alone and the script will detect the graphic server, --wayland if you have wayland in your computer or --x11.
```bash
cd <easynav_playgrounds>/playground_summit/easynav_playground_summit/docker/

./launch.sh
```
 
This script:

- Temporarily allows the Docker the access to the graphic server  (`xhost +local:docker`).
- Launch the docker allowing to see the Gazebo and RVIZ windows.
- When exiting Docker, revoke access to the graphical server.

### Option B: in our terminal
 
```bash
cd <easynav_playgrounds>/playground_summit/easynav_playground_summit/docker/

xhost +local:docker
 
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  --device /dev/dri \
  --name playground_summit \
  --gpus all \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  easynav_playground:summit_rolling
 
xhost -local:docker
```
