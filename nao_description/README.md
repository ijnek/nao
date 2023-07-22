# nao_description

## Setup

To install the 3d model meshes for the NAO, from your ROS workspace, run:


    ./src/nao/nao_description/install.sh

and step through the installation wizard.

## Using in a launch file

    file_path = os.path.join(
        get_package_share_directory('nao_description'),
            'urdf',
            'nao.urdf')
    with open(file_path, 'r') as infp:
        robot_description = infp.read()
