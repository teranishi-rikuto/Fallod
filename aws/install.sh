#! /bin/bash

sudo apt update
sudo apt install -y \
    python3-pip

git config --local user.name "sia"
git config --local user.email hello@world.com

pip3 install -r requirements.txt
