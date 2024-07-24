#!/bin/bash

# Activate Conda environment
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate py3

# Run the Python script from the transformers workspace
python3 "$(dirname "$0")/../../transformer/src/rosbridge/create_rosbridge_srv.py"

