xhost +local:
gui="--runtime=nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro"
docker run -v /ssdata/datasets/cityscapes/full:/cityscapes  $gui -it slamantic /bin/bash