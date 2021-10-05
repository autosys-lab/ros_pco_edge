#!/usr/bin/env bash

branch=$(git branch --show-current)

echo -e "Building pco edge driver for $branch\n WARNING: this script must be run from the root of the repo not from within the .docker folder"

docker build --pull --rm -f ./.docker/Dockerfile  -t gdwyer/ros_pco_edge:$branch-amd64 .
