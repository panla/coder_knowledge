import argparse
import re
import os
import sys
import shutil
import tempfile
import traceback
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from distutils.command.build_py import build_py
from distutils.core import setup

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


class FileOperator():
    def __init__(self, path: str) -> None:
        self.path = path

    def readlines(self):
        with open(self.path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            return [l.strip() for l in lines if l.strip()]


class DirContext():

    def __enter__(self):
        self.temp = tempfile.mkdtemp()
        return self.temp

    def __exit__(self, exc_type, exc_value, traceback):
        shutil.rmtree(self.temp)


def get_args():
    """获取命令行输入的参数"""

    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--src', required=True, type=str, help='源文件夹')
    parser.add_argument('-d', '--dest', required=True, type=str, help='目标文件夹')
    # parser.add_argument('-p', '--process', required=True, type=str, help='会被处理的文件规则文件')
    parser.add_argument('--cp_ignore', required=False, type=str, help='不会被复制的规则文件')
    parser.add_argument('--p_ignore', required=False, type=str, help='不会被处理的规则文件')
    params = parser.parse_args().__dict__

    src_dir = params.get('src')
    if not Path(src_dir).is_dir():
        # 退出，报错
        sys.stderr.write(f'参数 -o/--src 不存在\n\n')
        sys.exit(1)

    dest_dir = params.get('dest')

    cp_ignore_path = params.get('cp_ignore')
    if cp_ignore_path:
        if not Path(cp_ignore_path).is_file():
            # 退出，报错
            sys.stderr.write(f'参数 -c/--cp_ignore 不存在\n\n')
            sys.exit(1)
        cp_ignore_lis = FileOperator(cp_ignore_path).readlines()
    else:
        cp_ignore_lis = ["dist", ".git", "venv", ".idea", ".vscode", "tmp", "logs", "migrations", "__pycache__"]

    p_ignore_path = params.get('p_ignore')
    if p_ignore_path:
        if not Path(p_ignore_path).is_file():
            # 退出，报错
            sys.stderr.write(f'参数 -p/--p_ignore 不存在\n\n')
            sys.exit(1)
        p_ignore_lis = FileOperator(p_ignore_path).readlines()
    else:
        p_ignore_lis = ["__init__.py"]

    return src_dir, dest_dir, cp_ignore_lis, p_ignore_lis


def search(content, regexs):
    if isinstance(regexs, str):
        return re.search(regexs, content)

    for regex in regexs:
        if re.search(regex, content):
            return True


def copy_files(src_dir: Path, dest_dir: Path, ignore_lis: list):
    """复制源文件夹至目标文件夹"""

    if Path(dest_dir).is_dir():
        # 如果目标文件夹存在，就删除
        shutil.rmtree(dest_dir)

    def callable(src, names: list):
        # ???
        # if search(src, dest_dir):
            # return names
        return ignore_lis

    shutil.copytree(src_dir, dest_dir, ignore=callable)


def walk_dir(input_dir: Path):

    rt = []
    for root, _, file_names in os.walk(input_dir, topdown=False):
        for file_name in file_names:
            full_name = Path(root).joinpath(file_name)
            rt.append(full_name)
    return rt


def get_py_files(names: list, p_ignore_lis: list):
    for name in names:
        _name = name.__str__()
        if _name.endswith(('.py', '.pyx')) and not search(_name, p_ignore_lis):
            yield name


def transform_py_file(name: Path):
    try:
        file_name = name.name
        os.chdir(name.parent)

        with DirContext() as dc:

            setup(
                ext_modules=cythonize([file_name], compiler_directives=COMPILER_DIRECTIVES),
                script_args=['build_ext', '-t', dc, '--inplace']
            )
            sys.stdout.write(f'处理成功 {name}\n\n')

    except Exception as exc:

        sys.stderr.write(f'处理失败 {name}\n')
        sys.stderr.write(f'{traceback.format_exc()}\n\n')


def delete_files(file: Path):
    try:
        os.remove(file)

        if file.name.endswith('.py'):
            c_file = file.name.replace('.py', '.c')
        elif file.name.endswith('.pyx'):
            c_file = file.name.replace('.pyx', '.c')
        else:
            pass

        # os.remove(c_file)

    except Exception as exc:
        print(traceback.format_exc())


def execute():
    """执行入口"""

    src_dir, dest_dir, cp_ignore_lis, p_ignore_lis = get_args()
    src_dir = Path(src_dir).absolute()
    dest_dir = Path(dest_dir).absolute()

    # copy
    copy_files(src_dir, dest_dir, cp_ignore_lis)

    # list files
    py_files = walk_dir(dest_dir)
    # list need process files
    py_files = get_py_files(py_files, p_ignore_lis)

    # process
    for file in py_files:
        with ProcessPoolExecutor(max_workers=4) as exector:
            exector.submit(transform_py_file, file)

        delete_files(file)


if __name__ == "__main__":
    execute()
