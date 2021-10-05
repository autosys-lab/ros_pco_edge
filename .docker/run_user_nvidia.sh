#!/usr/bin/env bash

branch=$(git branch --show-current)
echo -e "Starting up the user pco edge container for branch $branch and access to an nvidia gpu, this container will access to the users home directory and logged in as the user with their password and x sever access.\nYou will not own the workspace though, use sudo chown -R \$UID /dev_ws"

docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --gpus 'all,"capabilities=compute,display,graphics,utility"' \
    gdwyer/ros_pco_edge:$branch-amd64