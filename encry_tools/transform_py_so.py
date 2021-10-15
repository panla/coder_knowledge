import argparse
import os
import sys
import shutil
import tempfile
import traceback
from distutils.command.build_py import build_py
from distutils.core import setup

from Cython.Build import cythonize


def get_package_dir(*args, **kwargs):
    return ""


# TODO ? 重写get_package_dir， 否则生成的so文件路径有问题
build_py.get_package_dir = get_package_dir


def start():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--root_dir', required=True, type=str, help='根目录')
    parser.add_argument('-n', '--name', required=True, type=str, help='文件夹/文件名称')
    parse_args = parser.parse_args().__dict__

    ROOT_DIR = parse_args.get('root_dir')
    if not os.path.isdir(ROOT_DIR):
        sys.stderr.write(f'参数 -i/--root_dir {ROOT_DIR} 不存在\n')
        sys.exit(1)

    NAME = parse_args.get('name')

    execute(ROOT_DIR, NAME)


class DirContext:
    def __enter__(self):
        self.name = tempfile.mkdtemp()
        return self.name

    def __exit__(self, exc_type, exc_value, traceback):
        shutil.rmtree(self.name)


def delete_files(py_file: str):
    """删除原 .py .pyx 文件"""

    try:
        os.remove(py_file)

        if py_file.endswith('.py'):
            c_file = py_file.replace('.py', '.c')
        else:
            c_file = py_file.replace('.pyx', '.c')
        if os.path.exists(c_file):
            os.remove(c_file)
            pass
    except Exception as exc:
        pass


def transform_py_file(py_file):
    try:
        file_name = os.path.basename(py_file)

        with DirContext() as dc:

            # TODO language_level=3 ?
            setup(
                ext_modules=cythonize(
                    [file_name],
                    quiet=True,
                    compiler_directives={'language_level': 3, 'always_allow_keywords': True}),
                script_args=['build_ext', '-t', dc, '--inplace']
            )

            sys.stdout.write(f'处理成功 {py_file}\n')

            delete_files(py_file)

    except Exception as exc:

        sys.stderr.write(f'处理失败 {py_file}\n')
        sys.stderr.write(f'{traceback.format_exc()}\n')

        delete_files(py_file)

    return py_file


def filter_files(name: str):
    if name.endswith('.py') and name not in ['__pycache__', '__init__.py']:
        return name


def execute(root_dir: str, name: str):
    """
    输入根目录
    输入一个文件路径或者文件夹路径
    """

    full_path = os.path.join(root_dir, name)
    if os.path.isdir(full_path):
        # 是目录
        os.chdir(full_path)
        for file_name in os.listdir(full_path):
            file_name = filter_files(file_name)
            if file_name:
                py_file = os.path.join(full_path, file_name)
                transform_py_file(py_file)
    if os.path.isfile(full_path):
        # 是文件
        os.chdir(root_dir)
        transform_py_file(full_path)


if __name__ == '__main__':
    start()
