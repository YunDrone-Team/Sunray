#!/bin/bash

# Exit immediately if a command fails
set -e

# Define environment name
ENV_NAME="NavRL"

# Load Conda environment handling
eval "$(conda shell.bash hook)"
conda create -n $ENV_NAME python=3.10

# Step 1: Setup Orbit
echo "Setting up Orbit..."
# cd ../orbit
cd ./third_party/orbit

# Remove existing symbolic link if it exists
if [ -L "_isaac_sim" ]; then
    echo "Removing existing symbolic link: _isaac_sim"
    rm -rf _isaac_sim
elif [ -e "_isaac_sim" ]; then
    echo "Error: _isaac_sim exists but is not a symlink. Remove it manually."
    exit 1
fi
ln -s ${ISAACSIM_PATH} _isaac_sim
echo "Running orbit.sh setup..."
./orbit.sh --conda $ENV_NAME
conda activate $ENV_NAME
pip install numpy==1.26.4
pip install "pydantic!=1.7,!=1.7.1,!=1.7.2,!=1.7.3,!=1.8,!=1.8.1,<2.0.0,>=1.6.2"
pip install imageio-ffmpeg==0.4.9
pip install moviepy==1.0.3
pip install pyyaml
pip install rospkg
pip install matplotlib

# Step 2: Install dependencies
echo "Installing system dependencies..."
sudo apt update && sudo apt install -y cmake build-essential

# Install Orbit dependencies
echo "Installing Orbit dependencies..."
./orbit.sh --install
# ./orbit.sh --extra


# Step 3: Navigate to OmniDrones directory
echo "Setting up OmniDrones..."
cd ../OmniDrones
cp -r conda_setup/etc $CONDA_PREFIX

# Re-activate the environment
conda activate $ENV_NAME

# Verification
echo "Verifying OmniIsaac Kit installation..."
python -c "from omni.isaac.kit import SimulationApp"

# Step 4: Setup OmniDrones package
echo "Setting up OmniDrones package..."
cd ../OmniDrones
pip install -e .

# Step 5: Install TensorDict and dependencies
echo "Installing TensorDict dependencies..."
pip uninstall -y tensordict
pip uninstall -y tensordict
pip install tomli  # If missing 'tomli'
cd ../tensordict
python setup.py develop


# Step 6: Install TorchRL
echo "Installing TorchRL..."
cd ../rl
python setup.py develop



# Check which torch is being used
python -c "import torch; print(torch.__path__)"

echo "Setup completed successfully!"

