"""
使用 cryptography 生成 RSA 公私钥
"""

from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives.serialization import Encoding, PrivateFormat, PublicFormat
from cryptography.hazmat.primitives.serialization import NoEncryption, BestAvailableEncryption

# RSA 私钥文件路径
private_key_file = './rsa_private_key.pem'
# RSA 公钥文件路径
public_key_file = './rsa_public_key.pem'


def write_file(data, path: str, is_byte: bool = True):
    """写入文件

    Args:
        data (bytes | str): 写入的内容
        path (str): 文件路径
        is_byte (bool, optional): 是否是 bytes 类型. Defaults to True.

    """

    if is_byte:
        with open(path, 'wb') as f:
            f.write(data)

    else:
        with open(path, 'w', encoding='utf-8') as f:
            f.write(data)


# 固定使用 65537
private_key = rsa.generate_private_key(
    public_exponent=65537,
    key_size=2048
)

public_key = private_key.public_key()

# 无密码私钥
private_key_data = private_key.private_bytes(
    encoding=Encoding.PEM, format=PrivateFormat.TraditionalOpenSSL, encryption_algorithm=NoEncryption()
)
# 带密码私钥
# private_key_data = private_key.private_bytes(
#     encoding=Encoding.PEM, format=PrivateFormat.TraditionalOpenSSL,
#     encryption_algorithm=BestAvailableEncryption(password='12345678')
# )
public_key_data = public_key.public_bytes(encoding=Encoding.PEM, format=PublicFormat.SubjectPublicKeyInfo)

write_file(private_key_data, private_key_file)
write_file(public_key_data, public_key_file)
