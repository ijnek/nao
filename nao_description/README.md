# nao_description

## Using in a launch file

    file_path = os.path.join(
        get_package_share_directory('nao_description'),
            'urdf',
            'nao.urdf')
    with open(file_path, 'r') as infp:
        robot_description = infp.read()
