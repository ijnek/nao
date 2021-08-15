#!/bin/bash

cd "$(dirname "$0")"
mkdir tmp
wget https://github.com/ros-naoqi/nao_meshes_installer/raw/master/naomeshes-0.6.7-linux-x64-installer.run -P tmp/
chmod +x tmp/naomeshes-0.6.7-linux-x64-installer.run && ./tmp/naomeshes-0.6.7-linux-x64-installer.run  --prefix . --mode text
mv -t ../  meshes/ texture/
cd -