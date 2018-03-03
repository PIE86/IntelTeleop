#!/bin/bash

# Add custom model path to GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$(rospack find display)/models:$GAZEBO_MODEL_PATH
