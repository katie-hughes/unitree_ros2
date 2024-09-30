#!/bin/bash
echo "Building docker image..."
docker build . --tag "unitree_ros2"
