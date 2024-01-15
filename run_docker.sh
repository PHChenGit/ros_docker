xhost +local:root
XAUTH=/tmp/.docker.xauth
# xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
# if [ ! -f $XAUTH ]
# then
#     echo XAUTH file does not exist. Creating one...
#     touch $XAUTH
#     chmod a+r $XAUTH
#     if [ ! -z "$xauth_list" ]
#     then
#         echo $xauth_list | xauth -f $XAUTH nmerge -
#     fi
# fi

# # Prevent executing "docker run" when xauth failed.
# if [ ! -f $XAUTH ]
# then
#   echo "[$XAUTH] was not properly created. Exiting..."
#   exit 1
# fi

docker run --rm -it \
    --gpus=all \
    --privileged \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --volume="$XAUTH:$XAUTH" \
    --env="XAUTHORITY=$XAUTH" \
    --env="DISPLAY=unix$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --net=host \
    ros:1.1-cudagl11.3.0-devel-ubuntu20.04 /bin/bash
