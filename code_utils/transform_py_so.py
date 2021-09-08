import argparse
import os
import sys
import re
import shutil
import tempfile
import traceback
from distutils.command.build_py import build_py
from distutils.core import setup
from typing import Union

from Cython.Build import cythonize


def get_package_dir(*args, **kwargs):
    return ""


# TODO ? 重写get_package_dir， 否则生成的so文件路径有问题
build_py.get_package_dir = get_package_dir


def execute():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--src_dir', required=True, type=str, help='源文件夹')
    parser.add_argument('-o', '--out_dir', required=True, type=str, help='目标文件夹')
    parser.add_argument('-r', '--rules', required=False, type=str, nargs='*', help='忽略规则')
    parse_args = parser.parse_args().__dict__

    SRC_DIR = parse_args.get('src_dir')
    SRC_DIR = os.path.abspath(SRC_DIR)
    if not os.path.isdir(SRC_DIR):
        sys.stderr.write(f'参数 -i/--src_dir {SRC_DIR} 不存在\n')
        sys.exit(1)

    OUT_DIR = parse_args.get('out_dir')
    OUT_DIR = os.path.abspath(OUT_DIR)
    RULES = parse_args.get('rules')

    start(SRC_DIR, OUT_DIR, RULES)


class DirContext:
    def __enter__(self):
        self.name = tempfile.mkdtemp()
        return self.name

    def __exit__(self, exc_type, exc_value, traceback):
        shutil.rmtree(self.name)


def search_files_by_rules(content, regexs: Union[str, list]):
    """通过正则过滤文件"""

    if isinstance(regexs, str):
        return re.search(regexs, content)

    for regex in regexs:
        if re.search(regex, content):
            return True


def copy_files(src_dir: str, dst_dir: str, ignore_rules: list):
    """复制源文件到目标位置

    忽略掉 '.git', '.vscode', '.idea', 'tmp', 'logs', '__pycache__', 'venv', '.gitignore'
    """

    if os.path.isdir(src_dir):
        if os.path.exists(dst_dir):
            sys.stdout.write(f'{dst_dir} 已存在，删除\n')
            shutil.rmtree(dst_dir)

        def callable(src, names: list):
            if search_files_by_rules(src, dst_dir):
                return names
            return ignore_rules

        shutil.copytree(src=src_dir, dst=dst_dir, ignore=callable)
    else:
        sys.stderr.write(f'{src_dir} 不存在\n')


def walk_dir(root_dir: str):
    for root, _, files in os.walk(root_dir):
        for file in files:
            yield os.path.join(root, file)


def read_file(path: str):
    with open(path, 'r', encoding='utf-8') as f:
        data = f.read()
    return data


def filter_py_files(files) -> list:
    """搜索 .py 文件

    搜索要被处理的文件
    """

    rt = []
    for file in files:
        if file.endswith('.py') and not file.endswith('__init__.py') and 'pydantic' not in read_file(file):
            rt.append(file)
    return rt


def delete_files(files: list):
    try:
        for py_file in files:
            os.remove(py_file)
            c_file = py_file.replace('.py', '.c')
            if os.path.exists(c_file):
                os.remove(c_file)
    except Exception as exc:
        pass


def rename_excrypted_file(files):
    for file in files:
        if file.endswith(".pyd") or file.endswith(".so"):
            new_filename = re.sub("(.*)\..*\.(.*)", r"\1.\2", file)
            os.rename(file, new_filename)


def encrypt_py_files(files: list) -> list:
    had_encrypted_files = []

    with DirContext() as dc:
        count = len(files)
        for index, py_file in enumerate(files):
            try:
                dir_name = os.path.dirname(py_file)
                file_name = os.path.basename(py_file)

                os.chdir(dir_name)
                sys.stdout.write(f'正在处理 {index + 1}/{count}, {file_name}\n')

                # TODO language_level=3 ?
                setup(
                    ext_modules=cythonize([file_name], quiet=True, language_level=3),
                    script_args=['build_ext', '-t', dc, '--inplace']
                )

                had_encrypted_files.append(py_file)
                sys.stdout.write(f'处理成功 {py_file}\n')
            except Exception as exc:

                sys.stderr.write(f'{traceback.format_exc()}\n')

                sys.stderr.write(f'处理失败 {py_file}\n')
                c_file = py_file.replace('.py', '.c')
                if os.path.exists(c_file):
                    os.remove(c_file)

    return had_encrypted_files


def start(src_dir: str, dst_dir: str, ignore_rules: list):
    if not ignore_rules:
        # 这些不会被复制
        ignore_rules = [
            '.git', '.vscode', '.idea', 'tmp', 'logs', '__pycache__', 'venv', '.gitignore', '.md'
        ]

    copy_files(src_dir, dst_dir, ignore_rules=ignore_rules)

    py_files = filter_py_files(walk_dir(dst_dir))

    had_encrypt_files = encrypt_py_files(py_files)

    delete_files(had_encrypt_files)

    rename_excrypted_file(walk_dir(dst_dir))

    sys.stdout.write(f'处理完成, total {len(py_files)} success {len(had_encrypt_files)}\n')


if __name__ == '__main__':
    execute()
