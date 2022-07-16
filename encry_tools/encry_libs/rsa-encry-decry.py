"""
使用 RSA 公钥加密-私钥解密，私钥签名-公钥验证
"""

from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding


# 原始文件路径
origin_file = './a.txt'
# 加密后的文件路径
encrypt_file = './a-encrypt.txt'
# 解密后的文件路径
decrypt_file = './a-decrypt.txt'
# RSA 私钥文件路径
private_key_file = './rsa_private_key.pem'
# RSA 公钥文件路径
public_key_file = './rsa_public_key.pem'


def read_file(path: str, is_byte: bool = True):
    """读取文件

    Args:
        path (str): 文件路径
        is_byte (bool, optional): 是否是 bytes 类型. Defaults to True.

    Returns:
        bytes | str: 文件内容
    """

    if is_byte:
        with open(path, 'rb') as f:
            data = f.read()
    else:
        with open(path, 'r', encoding='utf-8') as f:
            data = f.read()
    return data


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


# 加载私钥，产生私钥对象
private_key = serialization.load_pem_private_key(
    data=read_file(private_key_file), password=None, backend=default_backend()
)
# 加载公钥，产生公钥对象
public_key = serialization.load_pem_public_key(data=read_file(public_key_file), backend=default_backend())

# 用公钥来加密，生成加密后的数据
encrypt_data = public_key.encrypt(
    plaintext=read_file(origin_file,),
    padding=padding.OAEP(
        mgf=padding.MGF1(algorithm=hashes.SHA256()),
        algorithm=hashes.SHA256(),
        label=None
    )
)
# 保存加密后的数据
write_file(encrypt_data, encrypt_file)

# 用私钥来解密，生成解密后的数据
decrypt_data = private_key.decrypt(
    ciphertext=encrypt_data,
    padding=padding.OAEP(
        mgf=padding.MGF1(algorithm=hashes.SHA256()),
        algorithm=hashes.SHA256(),
        label=None
    )
)
# 保存解密后的数据
write_file(decrypt_data, decrypt_file)

# 私钥签名数据
signature_data = private_key.sign(
    data=read_file(origin_file),
    padding=padding.PSS(
        mgf=padding.MGF1(hashes.SHA256()),
        salt_length=padding.PSS.MAX_LENGTH
    ),
    algorithm=hashes.SHA256()
)

# 公钥校验，校验失败的抛出错误
public_key.verify(
    signature=signature_data,
    data=read_file(origin_file),
    padding=padding.PSS(
        mgf=padding.MGF1(hashes.SHA256()),
        salt_length=padding.PSS.MAX_LENGTH
    ),
    algorithm=hashes.SHA256()
)
