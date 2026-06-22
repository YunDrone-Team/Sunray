#!/bin/bash

# Exit immediately if a command fails
set -e

# Define environment name
ENV_NAME="NavRL"

# Load Conda environment handling
eval "$(conda shell.bash hook)"


# Step 1: Create conda env with python3.10
echo "Setting up conda env..."
conda create -n $ENV_NAME python=3.10 -c conda-forge
conda activate $ENV_NAME
pip install numpy==1.26.4
pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2
pip install "pydantic!=1.7,!=1.7.1,!=1.7.2,!=1.7.3,!=1.8,!=1.8.1,<2.0.0,>=1.6.2"
pip install imageio-ffmpeg==0.4.9
pip install moviepy==1.0.3
pip install hydra-core --upgrade
pip install einops
pip install pyyaml
pip install rospkg
pip install matplotlib

# Step 2: Install TensorDict and dependencies
echo "Installing TensorDict dependencies..."
pip uninstall -y tensordict
pip uninstall -y tensordict
pip install tomli  # If missing 'tomli'
cd ./third_party/tensordict
python setup.py develop


# Step 3: Install TorchRL
echo "Installing TorchRL..."
cd ../rl
python setup.py develop

# Check which torch is being used
python -c "import torch; print(torch.__path__)"

echo "Setup completed successfully!"
