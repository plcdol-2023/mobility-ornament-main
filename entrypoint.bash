#!/bin/bash
apt update && apt upgrade -y 
apt install gcc libc6-dev -y
apt-get -y install libgl1-mesa-glx libglib2.0-0
pip3 install -r requirements.txt

exec /bin/bash