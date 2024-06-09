#!/bin/sh

source ~/projects/gap_sdk/sdk_env/bin/activate
source ~/projects/gap_sdk/configs/ai_deck.sh 

pip install numpy==1.23
pip install scikit-learn --upgrade

nntool

open pathfinder3.onnx

adjust

fusions --scale8

aquant drone_images_cropped/* -H 30 -W 324 --scheme SQ8 -D 255

imageformat input_1 bw8 offset_int8

save_state

nntool -g -M . -m model.c -T tensors -H model.h pathfinder3.json

