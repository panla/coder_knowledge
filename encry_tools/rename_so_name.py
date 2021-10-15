import argparse
import os
import re
import sys


def start():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--root_dir', required=True, type=str, help='根目录')
    parse_args = parser.parse_args().__dict__

    ROOT_DIR = parse_args.get('root_dir')
    if not os.path.isdir(ROOT_DIR):
        sys.stderr.write(f'参数 -i/--root_dir {ROOT_DIR} 不存在\n')
        sys.exit(1)

    execute(ROOT_DIR)


def rename_so_file(py_file: str):
    """重命名 .so 文件"""

    if py_file.endswith(".pyd") or py_file.endswith(".so"):
        new_filename = re.sub("(.*)\..*\.(.*)", r"\1.\2", py_file)
        os.rename(py_file, new_filename)


def walk_dir(root_dir: str):
    for root, _, files in os.walk(root_dir):
        for file in files:
            yield os.path.join(root, file)


def execute(root_dir: str):
    for file in walk_dir(root_dir):
        rename_so_file(file)


if __name__ == '__main__':
    start()
