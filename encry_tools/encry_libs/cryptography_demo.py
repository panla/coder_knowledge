########################################################################################################################
# cryptography 多次执行 encrypt 结果不同，用 decrypt 来比较
# 解密时应用加密时对应的秘钥
#   <---->  加密解密双向

from cryptography.fernet import Fernet


def func_1():
    """使用不同秘钥加密，秘钥不同，加密结果也不同"""

    origin = 'abcdefg'

    # 把原始数据转为 字节格式
    origin = bytes(origin, encoding='utf8')

    # 产生一个秘钥
    secret = Fernet.generate_key()
    # 利用秘钥生成 operator
    op = Fernet(secret)

    # 产生加密后的数据
    encrypt_data = op.encrypt(origin)

    print(origin)
    print(secret)
    print(encrypt_data)
    print(op.decrypt(encrypt_data))

    # 22 产生一个秘钥
    secret = Fernet.generate_key()
    # 利用秘钥生成 operator
    op = Fernet(secret)

    # 产生加密后的数据
    encrypt_data = op.encrypt(origin)

    print(origin)
    print(secret)
    print(encrypt_data)
    print(op.decrypt(encrypt_data))

    """
    b'abcdefg'
    b'yJbnD2L3Hva8vJlHoLcb1akPwhtGxFyO2nrhti9_F48='
    b'gAAAAABitn12y4Ty8_F1DhpE8vh9PiHwpQrxVJEIE967O9GA1_dc5bgNiaeZZpYnbyTFJv7cwNzdCFFUq63D4S5H20YQlL306A=='
    b'abcdefg'
    
    b'abcdefg'
    b'9oebXV2MXLgFMO7DW8aq-qTQjOoJKYF35OlWT3fMV80='
    b'gAAAAABitn12OSZMcnzRbHPRDQ8quqMTbxpVRlFs_7O7CH66pPWyUPfg_UEHpWRBOtKEpHpZ7khQx7loCTiTr8BYjVnMh_z9eA=='
    b'abcdefg'
    """


def func_2():
    """使用相同秘钥来加密，秘钥相同加密结果也不同"""

    origin = 'abcdefg'

    # 把原始数据转为 字节格式
    origin = bytes(origin, encoding='utf8')

    # 产生一个秘钥
    secret = Fernet.generate_key()
    # 利用秘钥生成 operator
    op = Fernet(secret)

    # 产生加密后的数据
    encrypt_data = op.encrypt(origin)

    print(origin)
    print(secret)
    print(encrypt_data)
    print(op.decrypt(encrypt_data))

    # 产生加密后的数据
    encrypt_data = op.encrypt(origin)

    print(origin)
    print(secret)
    print(encrypt_data)
    print(op.decrypt(encrypt_data))

    """
    b'abcdefg'
    b's8jsjr_7JU9lh1uW-BCoXiKunXfXE2-m4dft5YbKVGc='
    b'gAAAAABitn2aUfN8buOO6e8w5tpqn92qCy5CxvDpSWUFJebRipFwHHPbHc-bz6DLidKoXUr2vmfI0--F1vJj3JpV89wZtf1QAQ=='
    b'abcdefg'
    
    b'abcdefg'
    b's8jsjr_7JU9lh1uW-BCoXiKunXfXE2-m4dft5YbKVGc='
    b'gAAAAABitn2ais5OM5Rv0g8aC7b3kaHMHK2J2Prh5sdh_X-J46lsnHzltDGNfINF-GoZNYcDfEJQ8JqCxKwMGO-kqLV_9e3GsQ=='
    b'abcdefg'
    """


def func_3():
    """使用不同的秘钥对应解密"""

    origin = b'abcdefg'
    secret = b'uHXxEny7g88oZdflz8d-Xf3ij8vuzluwXiyKFqvbmas='
    hash_data = b'gAAAAABiIeWufSrBJIlCi6y3dREDVexCyNw_x_upeWna21evSCUXF17ug_6q4R9WlXnkPPvnEmwMljOKVdV6hoM7hBHGtaYZMQ=='

    op = Fernet(secret)
    print(op.decrypt(hash_data))

    secret = b'PQBD2Ta_MyJMStO2boI2EI4cvkTex-Es03Mjr_B6gns='
    hash_data = b'gAAAAABiIeXA1ydDHbSpiX2tcuaHgitGBFKCfltsSV7WkuOIwTvqGLXeuLJz2mybmkcLeUi16F622hSYFcISAiXQF8ttB_A5yw=='
    op = Fernet(secret)
    print(op.decrypt(hash_data))

    # 使用不同的密钥交叉解密 失败
    # secret = b'uHXxEny7g88oZdflz8d-Xf3ij8vuzluwXiyKFqvbmas='
    # hash_data = b'gAAAAABiIeXA1ydDHbSpiX2tcuaHgitGBFKCfltsSV7WkuOIwTvqGLXeuLJz2mybmkcLeUi16F622hSYFcISAiXQF8ttB_A5yw=='
    # op = Fernet(secret)
    # print(op.decrypt(hash_data))


def func_4():
    """使用相同的密钥加密多次，加密结果不同，使用该密钥解密，解密结果相同"""

    origin = b'abcdefg'
    secret = b'xbbDQD1AG8QGc6R8XjebyfpJDLRJxzJnQctYqIdYbOY='
    hash_data = b'gAAAAABiIedFlIRIw812M2RxiVwRJEKp7-RcAliLR883kIvNjc0joFrk_IhTnCXq0JbDm6pzhoHWp57LSgLeW1PJ2_Ic0UVgOQ=='

    op = Fernet(secret)
    print(op.decrypt(hash_data))

    hash_data = b'gAAAAABiIedF80FraAdBrMOy0VMDJ77eoNHwXZ_5vO0H42-eqJFffoO5K-ZbDM8WlbFbLP5rZlfglTebDleQMuADAMtCInaYMA=='
    print(op.decrypt(hash_data))


# func_1()
func_2()
# func_3()
# func_4()
