import yaml
if __name__ == '__main__':
    with open('config.yaml','r',encoding='utf-8') as file:
        config = yaml.safe_load(file)

    for launch_file in config['HUMAN_AI']:
        # 为每个机械臂构建独立的命令
        command = [
            'ros2', 'launch',
            'ros2_chess_bringup',
            launch_file['name']
        ]
        for parameter_name,parameter_value in launch_file['parameters'].items():
            command.append(f"{parameter_name}:={parameter_value}")

        print(command)