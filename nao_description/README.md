# nao_description

## Setup

To install the 3d model meshes for the NAO, in this nao_description package directory,

    mkdir tmp && cd tmp
    wget https://github.com/ros-naoqi/nao_meshes_installer/raw/master/naomeshes-0.6.7-linux-x64-installer.run
    chmod +x naomeshes-0.6.7-linux-x64-installer.run && ./naomeshes-0.6.7-linux-x64-installer.run  --prefix .
    *Follow instructions, don't change the install directory*
    mv -t ../  meshes/ texture/
    cd ../ && rm -rf tmp/

## Environment

Tested for ROS2 FOXY, on Ubuntu 20.04

## Using in a launch file

    file_path = os.path.join(
        get_package_share_directory('nao_description'),
            'urdf',
            'nao.urdf')
    with open(file_path, 'r') as infp:
        robot_description = infp.read()
