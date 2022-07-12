"""
使用 cryptography 生成 RSA 公私钥

用私钥来生成 X.509 证书
"""

from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives.serialization import Encoding, PrivateFormat, PublicFormat
from cryptography.hazmat.primitives.serialization import NoEncryption, BestAvailableEncryption

# RSA 私钥文件路径
private_key_file = './rsa_private_key.pem'
# RSA 公钥文件路径
public_key_file = './rsa_public_key.pem'
# 服务端证书请求文件路径
server_csr_file = './server-csr.pem'


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


# 私钥对象，固定使用 65537
private_key = rsa.generate_private_key(
    public_exponent=65537,
    key_size=2048
)

# 公钥对象
public_key = private_key.public_key()

# 无密码私钥数据
private_key_data = private_key.private_bytes(
    encoding=Encoding.PEM,
    format=PrivateFormat.TraditionalOpenSSL,
    encryption_algorithm=NoEncryption()
)
# 带密码私钥数据
# private_key_data = private_key.private_bytes(
#     encoding=Encoding.PEM, format=PrivateFormat.TraditionalOpenSSL,
#     encryption_algorithm=BestAvailableEncryption(password='12345678')
# )

# 公钥数据
public_key_data = public_key.public_bytes(
    encoding=Encoding.PEM,
    format=PublicFormat.SubjectPublicKeyInfo
)

write_file(private_key_data, private_key_file)
write_file(public_key_data, public_key_file)

"""
# 待定

from datetime import datetime, timedelta
from ipaddress import IPv4Address
from time import time

from cryptography import x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.primitives import hashes

subject_name = issuer = x509.Name([
    x509.NameAttribute(NameOID.COUNTRY_NAME, u"US"),
    x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, u"California"),
    x509.NameAttribute(NameOID.LOCALITY_NAME, u"San Francisco"),
    x509.NameAttribute(NameOID.ORGANIZATION_NAME, u"My Company"),
    x509.NameAttribute(NameOID.COMMON_NAME, u"Root"),
])

ip_address = [
    x509.IPAddress(IPv4Address('192.168.9.90')),
    x509.IPAddress(IPv4Address('192.168.9.96')),
    x509.IPAddress(IPv4Address('192.168.9.99')),
    x509.IPAddress(IPv4Address('192.168.9.114')),
]
cert = x509.CertificateBuilder(
    ).subject_name(
        name=subject_name
    ).issuer_name(
        name=issuer
    ).public_key(
        key=public_key
    ).serial_number(
        number=x509.random_serial_number()
    ).not_valid_before(
        time=datetime.utcnow()
    ).not_valid_after(
        time=datetime.utcnow() + timedelta(days=365)
    ).add_extension(
        extval=x509.SubjectAlternativeName(general_names=ip_address),
        critical=False
    ).sign(
        private_key=private_key,
        algorithm=hashes.SHA256()
    )


csr_data = cert.public_bytes(encoding=Encoding.PEM)

write_file(csr_data, server_csr_file)
"""
