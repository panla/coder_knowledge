import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument('-d', '--dir', type=str, required=True, help='文件夹路径')
parser.add_argument('-a', '--aim', type=str, required=True, help='目标')

params = parser.parse_args()
input_dir = params.dir
input_aim = params.aim

if not os.path.isdir(input_dir):
    sys.stderr.write('输入的参数 -d/--dir 不存在\n')
    sys.exit(1)
input_dir = os.path.abspath(input_dir)

for root, dirs, file_names in os.walk(input_dir):
    for file_name in file_names:
        path = os.path.join(root, file_name)
        try:
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
                if input_aim in content:
                    sys.stdout.write(f'{path}\n')
        except:
            pass

sys.stdout.write('查找结束\n')
