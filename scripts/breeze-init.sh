#!/bin/bash

# update apt packages
sudo apt update
sudo apt upgrade -y

# install apt package dependencies
sudo apt install --no-install-recommends git cmake ninja-build gperf ccache dfu-util device-tree-compiler wget \
     python3-dev python3-venv python3-tk python-is-python3 xz-utils file make gcc g++ gdb-multiarch \
     gcc-multilib g++-multilib libsdl2-dev libmagic1 openocd -y

# create workspace
mkdir breeze-workspace
cd breeze-workspace/

# init python venv
python -m venv .venv
source .venv/bin/activate

# install west
pip install west

# install breeze
west init -m https://github.com/RobotPilots-SZU/breeze.git .
west update

#install pip package dependencies
west packages pip --install
source breeze-sdk/zephyr/zephyr-env.sh

# copy environment initial file
cp breeze/scripts/env.sh env.sh
