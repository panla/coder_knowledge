import json
import os
import re
import shutil
import tempfile
import traceback
from setuptools import setup
from setuptools.command.build_py import build_py

import click
from Cython.Build import cythonize


# 编译指令
COMPILER_DIRECTIVES = {
    'language_level': 3,
    'always_allow_keywords': True,
    'annotation_typing': False
}

# 重写get_package_dir， 否则生成的so文件路径有问题
def get_package_dir(*args, **kwargs):
    return ""

build_py.get_package_dir = get_package_dir


@click.group()
@click.pass_context
def cli(ctx: click.Context):
    pass


class DirContext():

    def __enter__(self):
        self.temp = tempfile.mkdtemp()
        return self.temp

    def __exit__(self, exc_type, exc_value, traceback):
        shutil.rmtree(self.temp)


class BuildOperator:

    def __init__(self, origin_dir: str, target_dir: str, rules: dict) -> None:
        self.origin_dir = origin_dir
        self.target_dir = target_dir
        self.rules = rules

    @staticmethod
    def read_json(path: str):
        """读取 JSON 文件"""

        with open(path, 'r', encoding='utf-8') as f:
            data = json.loads(f.read())
        return data

    def check_re_rules(self, name, rules: list):
        """检查文件名是否符合相关规则"""

        for rule in rules:
            if re.findall(rule, name):
                return False
        return True

    def copy(self, origin_full_path: str):
        """复制指定文件到指定位置"""

        # 原始文件相对于原始文件夹的相对路径
        origin_abs_path = origin_full_path.replace(self.origin_dir + '/', '')
        # 目标文件全路径
        target_full_path = os.path.join(self.target_dir, origin_abs_path)

        # 目标文件的父级文件夹路径
        target_file_dirname = os.path.dirname(target_full_path)
        # 创建文件夹
        os.makedirs(target_file_dirname, exist_ok=True)
        # 复制
        shutil.copyfile(origin_full_path, target_full_path)

    def find_and_copy(self):
        """查找并复制相关文件"""

        for top_name in os.listdir(self.origin_dir):
            if top_name not in self.rules.get('not_copy'):

                top_full_path = os.path.join(self.origin_dir, top_name)
                if os.path.isfile(top_full_path):
                    self.copy(top_full_path)

                for root, _, names in os.walk(top_full_path):
                    for name in names:
                        if self.check_re_rules(name, self.rules.get('not_copy_re_files')):
                            # 原始文件全路径
                            origin_full_path = os.path.join(root, name)
                            self.copy(origin_full_path)

    @staticmethod
    def delete_file(full_path: str, flag: int):
        """删除文件

        file 完整的 .py .pyx 文件路径
        """

        try:
            os.remove(full_path)

            if full_path.endswith('.py'):
                c_file = full_path.replace('.py', '.c')
            elif full_path.endswith('.pyx'):
                c_file = full_path.replace('.pyx', '.c')
            else:
                pass

            if flag == 1:
                os.remove(c_file)

        except Exception as exc:
            print(traceback.format_exc())

    @staticmethod
    def rename_so_file(full_path: str):
        """重命名 so 文件"""

        new_filename = re.sub("(.*)\..*\.(.*)", r"\1.\2", full_path)
        os.rename(full_path, new_filename)

    def transform_py_file(self, full_path: str):
        """转换为 .so 的核心 code"""

        try:
            name = os.path.basename(full_path)
            with DirContext() as dc:

                setup(
                    ext_modules=cythonize([name], quiet=True, compiler_directives=COMPILER_DIRECTIVES),
                    script_args=['build_ext', '-t', dc, '--inplace'],
                    zip_safe=False
                )
                print(f'处理成功 {full_path}\n')

        except Exception as exc:

            print(f'处理失败 {full_path}')
            print(f'{traceback.format_exc()}\n')

    def execute(self):
        """执行入口"""

        for root, _, names in os.walk(self.target_dir):
            for name in names:
                if name.endswith('.py'):
                    if self.check_re_rules(name, self.rules.get('not_build_files')):
                        os.chdir(root)

                        full_path = os.path.join(root, name)
                        self.transform_py_file(full_path)
                        self.delete_file(full_path, flag=2)

        for root, _, names in os.walk(self.target_dir):
            for name in names:
                full_path = os.path.join(root, name)
                if re.findall('build\/lib\.linux-', full_path) and '.so' in full_path:
                    os.remove(full_path)
                    os.removedirs(os.path.dirname(full_path))

        for root, _, names in os.walk(self.target_dir):
            for name in names:
                if name.endswith('.so'):
                    full_path = os.path.join(root, name)
                    self.rename_so_file(full_path)


@click.command(help='transform py to so')
@click.pass_context
@click.option('-f', '--config_file', type=str, required=True, help='config json file')
def build(ctx: click.Context, config_file: str):
    """把 Python 代码转换为 .so

    config_file:
        {
            "origin_dir": "",
            "target_dir": "",
            "rules": {
                "not_copy": ["docs", "tmp", ".git", "__pycache__", "README.md", "CHANGELOG.md", ".gitignore"],
                "not_copy_re_files": ["(.*?)\.pyc"],
                "not_build_files": ["__init__.py", "server.py"]
            }
        }
    """

    config_params = BuildOperator.read_json(config_file)

    origin_dir = config_params.get('origin_dir')
    target_dir = config_params.get('target_dir')
    rules = config_params.get('rules')

    build_op = BuildOperator(origin_dir=origin_dir, target_dir=target_dir, rules=rules)

    build_op.find_and_copy()

    build_op.execute()

cli.add_command(build)

if __name__ == '__main__':
    cli()
