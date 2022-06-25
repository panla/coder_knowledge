########################################################################################################################
# hashlib 多次执行生成的值相同，解析后直接比较

import binascii
from hashlib import pbkdf2_hmac

# 原始数据
origin = bytes('12345678', encoding='utf8')
# 加盐
salt = bytes('abcdefg', encoding='utf8')
# 迭代次数
iteration = 100

x = pbkdf2_hmac(hash_name='sha256', password=origin, salt=salt, iterations=iteration)
print(binascii.hexlify(x).decode())
# e7ce9de3410047c5472b8ebb9de491a6b5bdbfd98029e94ae9ebfb9c7085f5d3
