#!/bin/bash

sudo apt-get update
sudo apt-get install i2c-tools
sudo apt-get install python-smbus
cd ~/
git clone https://github.com/acadams/Freenove_Three-wheeled_Smart_Car_Kit_for_Raspberry_Pi.git
sudo apt-get install libv4l-dev
sudo apt-get install libjpeg8-dev
sudo apt-get install imagemagick
sudo apt-get update
sudo apt-get install python-qt4
sudo apt-get install python-dev

alias gs='git status'
alias pull='git pull --rebase'