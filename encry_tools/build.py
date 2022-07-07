"""
使用
from setuptools.command.build_py import build_py
from setuptools import setup
时

setup 加上 zip_safe=False
"""

import argparse
import os
import sys
import shutil
import tempfile
import traceback
from pathlib import Path
from distutils.command.build_py import build_py
# from setuptools.command.build_py import build_py
from distutils.core import setup
# from setuptools import setup

from Cython.Build import cythonize

# 编译指令
COMPILER_DIRECTIVES = {
    'language_level': 3,
    'always_allow_keywords': True,
    'annotation_typing': False
}


def get_package_dir(*args, **kwargs):
    return ""


# TODO ? 重写get_package_dir， 否则生成的so文件路径有问题
build_py.get_package_dir = get_package_dir


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--root_dir', required=True, type=str, help='根目录')
    parser.add_argument('-n', '--name', required=True, type=str, help='文件夹/文件名称')
    parser.add_argument('--delete', default=1, required=False, type=int, help='是否删除C文件，默认删除')
    parser.add_argument('--ignore', required=False, type=str, nargs='*', help='忽略规则')
    parse_args = parser.parse_args().__dict__

    ROOT_DIR = parse_args.get('root_dir')
    NAME = parse_args.get('name')
    DELETE = parse_args.get('delete')
    IGNORE_RULES = parse_args.pop('ignore')
    if not IGNORE_RULES:
        IGNORE_RULES = ['__pycache__', '__init__.py', '__init__.pyx', '.pyc']

    TransformOperator(ROOT_DIR, NAME, IGNORE_RULES, DELETE).execute()


class DirContext():

    def __enter__(self):
        self.temp = tempfile.mkdtemp()
        return self.temp

    def __exit__(self, exc_type, exc_value, traceback):
        shutil.rmtree(self.temp)


class FileOperator():

    def __init__(self, name: str) -> None:
        self.name = name

    def delete_file(self, flag: int):
        """删除文件

        file 完整的 .py .pyx 文件路径
        """

        try:
            os.remove(self.name)

            if self.name.endswith('.py'):
                c_file = self.name.replace('.py', '.c')
            elif self.name.endswith('.pyx'):
                c_file = self.name.replace('.pyx', '.c')
            else:
                pass

            if flag == 1:
                os.remove(c_file)

        except Exception as exc:
            print(traceback.format_exc())

    def check(self, name, ignore_rules: list = None):
        """ TODO 检查传入的文件、文件夹是否符合规则"""

        flag = True

        for ignore_rule in ignore_rules:
            if ignore_rule in name:
                flag = False
        return flag

    def find_files(self, ignore_rules: list = None):
        """传入一个 文件夹或文件名称，返回符合要求的文件路径列表"""

        rt = []
        if Path(self.name).is_dir():
            for file in os.listdir(self.name):
                if Path(file).suffix in ['.py', '.pyx'] and self.check(file, ignore_rules):
                    rt.append(os.path.join(self.name, file))
        elif Path(self.name).is_file():
            if Path(self.name).suffix in ['.py', '.pyx'] and self.check(self.name, ignore_rules):
                rt.append(self.name)
        else:
            raise Exception(f'输入的参数 name {self.name} 不符合需求')
        return rt


class TransformOperator():
    def __init__(self, root_dir: str, name: str, ignore_rules: list, delete: int) -> None:
        self.root_dir = root_dir
        self.name = name
        self.ignore_rules = ignore_rules
        self.delete = delete

    def transform_py_file(self, name):
        try:
            file_name = os.path.basename(name)

            with DirContext() as dc:

                setup(
                    ext_modules=cythonize([file_name], quiet=True, compiler_directives=COMPILER_DIRECTIVES),
                    script_args=['build_ext', '-t', dc, '--inplace']
                )
                sys.stdout.write(f'处理成功 {name}\n\n')

        except Exception as exc:

            sys.stderr.write(f'处理失败 {name}\n')
            sys.stderr.write(f'{traceback.format_exc()}\n\n')

    def execute(self):
        file_path = os.path.join(self.root_dir, self.name)
        if Path(file_path).is_file():
            os.chdir(self.root_dir)
        elif Path(file_path).is_dir():
            os.chdir(file_path)
        else:
            raise Exception(f'输入的参数 name {self.name} 不符合需求')
        for file in FileOperator(file_path).find_files(ignore_rules=self.ignore_rules):

            self.transform_py_file(file)
            FileOperator(file).delete_file(self.delete)

if __name__ == '__main__':
    main()
