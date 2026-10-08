#!/bin/bash
set -e

IMAGE_NAME="easynav_playground:summit_rolling"
CONTAINER_NAME="playground_summits"
ROS_DOMAIN_ID="0"

# Select the graphics option.
# Automatic detection is performed using $XDG_SESSION_TYPE,
# but it can also be selected explicitly with --wayland or --x11.
SESSION_TYPE="auto"

usage() {
    echo "Usage: $0 [--wayland | --x11 | --auto]"
    echo "  --wayland   Enable NVIDIA PRIME render offload (for laptops with"
    echo "              hybrid Intel/NVIDIA GPUs running under Wayland)."
    echo "  --x11       Do not use PRIME render offload."
    echo "  --auto      Automatically detect the session type using \$XDG_SESSION_TYPE."
    exit 1
}

for arg in "$@"; do
    case "$arg" in
        --wayland) SESSION_TYPE="wayland" ;;
        --x11)     SESSION_TYPE="x11" ;;
        --auto)    SESSION_TYPE="auto" ;;
        -h|--help) usage ;;
        *) echo "Unknown option: $arg"; usage ;;
    esac
done

if [ "$SESSION_TYPE" = "auto" ]; then
    SESSION_TYPE="${XDG_SESSION_TYPE:-x11}"
fi

GPU_ENV_FLAGS=()
if [ "$SESSION_TYPE" = "wayland" ] && command -v nvidia-smi &>/dev/null; then
    echo "Wayland + NVIDIA GPU session detected: using PRIME render offload."
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only
    GPU_ENV_FLAGS+=(-e __NV_PRIME_RENDER_OFFLOAD -e __GLX_VENDOR_LIBRARY_NAME -e __VK_LAYER_NV_optimus)
fi

# Allows the Docker the access to the graphic server X11
xhost +local:docker

docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    --device /dev/dri \
    --name "$CONTAINER_NAME" \
    --gpus all \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    "$IMAGE_NAME"


# Revoke access to the graphical server
xhost -local:docker