"""一个 对文件夹内文件加密与解密的工具

所需第三方库 pip install cryptography

usage
    python encry.py --help

加密混淆
    python encry.py --origin /srv/api/apps --target /srv/api/apps2 --secret ./s.txt --type 1

解密还原
    python encry.py --origin /srv/api/apps2 --target /srv/api/apps3 --secret ./s.txt --type 2

TODO 增加对忽略文件的支持

"""

import argparse
import os
import sys

from cryptography.fernet import Fernet


parser = argparse.ArgumentParser()
parser.add_argument('--origin', required=True, type=str, help='原始文件夹')
parser.add_argument('--target', required=True, type=str, help='目标文件夹')
parser.add_argument('--type', required=True, type=int, help='1 加密 2 解密')
parser.add_argument('--secret', required=True, type=str, help='秘钥')

params = parser.parse_args().__dict__

origin_dir = params.get('origin')
target_dir = params.get('target')
action_type = params.get('type')
secret_file = params.get('secret')

if not os.path.isdir(origin_dir):
    sys.stderr.write(f'参数 --origin {origin_dir} 不存在')
    sys.exit(1)

if action_type not in [1, 2]:
    sys.stderr.write(f'参数 --action_type {action_type} 错误')
    sys.exit(1)

if not os.path.isfile(secret_file):
    sys.stderr.write(f'参数 --secret {secret_file} 不存在')
    sys.exit(1)

os.makedirs(target_dir)


class FileOperator:
    def __init__(self, path: str) -> None:
        self.path = path

    def read_binray_file(self):
        with open(self.path, 'rb') as fi:
            return fi.read()

    def write_binray_file(self, data):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)

        with open(self.path, 'wb') as fi:
            fi.write(data)

    def write_file(self, data):
        with open(self.path, 'w') as fi:
            fi.write(data)

    def walk(self):
        """遍历获得所有文件路径"""

        for root, _, names in os.walk(self.path):
            for name in names:
                path = os.path.join(root, name)
                yield path


class CryptOp:
    def __init__(self, secret) -> None:
        self.op = Fernet(secret)

    def encrypt(self, origin_data) -> bytes:
        """加密

        encrypted_data: bytes
        secret: bytes
        """

        encrypted_data = self.op.encrypt(origin_data)
        return encrypted_data

    def decrypt(self, encrypted_data) -> bytes:
        """解密

        encrypted_data: bytes
        """

        decrypted_data = self.op.decrypt(encrypted_data)
        return decrypted_data


def main():
    secret = FileOperator(secret_file).read_binray_file()

    if action_type == 1:
        # 加密

        for file_path in FileOperator(origin_dir).walk():
            new_file_path = file_path.replace(origin_dir, target_dir)

            origin_data = FileOperator(file_path).read_binray_file()

            encrypted_data = CryptOp(secret=secret).encrypt(origin_data)

            FileOperator(new_file_path).write_binray_file(encrypted_data)

    if action_type == 2:
        # 解密
        for file_path in FileOperator(origin_dir).walk():
            new_file_path = file_path.replace(origin_dir, target_dir)

            origin_data = FileOperator(file_path).read_binray_file()

            decrypted_data = CryptOp(secret=secret).decrypt(origin_data)

            FileOperator(new_file_path).write_binray_file(decrypted_data)


if __name__ == '__main__':
    main()
