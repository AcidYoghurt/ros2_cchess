import os
import yaml


if __name__ == '__main__':
    with open('machinery.yaml', 'r', encoding='utf-8') as file:
        config_file = yaml.safe_load(file)

    print(type(config_file['/**']['ros__parameters']['origin_position']))
    print(config_file['/**']['ros__parameters']['origin_position'])