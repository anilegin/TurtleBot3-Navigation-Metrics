# Create maps directory if it doesn't exist
if (!(Test-Path "maps")) {
    New-Item -ItemType Directory -Force -Path "maps"
}

# Run container with Xming configuration
# docker run -it --rm \
#     --privileged \
#     -e DISPLAY=$DISPLAY \
#     -e LIBGL_ALWAYS_SOFTWARE=1 \
#     -e QT_X11_NO_MITSHM=1 \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v $(pwd):/root/tbot3_ws \
#     --network host \
#     tbot3_humble

docker run -it -d \
  --name tbot3_container \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/root/tbot3_ws \
  --network host \
  tbot3_humble