#!/bin/bash

cat score.yaml | rostopic pub /picker/score std_msgs/Float32MultiArray
