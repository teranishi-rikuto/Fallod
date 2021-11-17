#! /bin/bash

sudo apt update
sudo apt install -y \
    python3-pip

git config --global user.name "urasaki"
git config --global user.email github@example.com

pip3 install -r requirements.txt
