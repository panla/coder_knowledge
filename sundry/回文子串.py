def is_palindrome1(s):
    """
    """
    if not isinstance(s, str):
        raise ValueError('Please input a string!')
    front, back = 0, len(s)-1
    while front < back:
        if s[front] != s[back]:
            return False
        front += 1
        back -= 1
    return True


def is_palindrome2(s):
    """
    """
    if not isinstance(s, str):
        raise ValueError('Please input a string!')
    if len(s) % 2 == 0:
        # Python 3.X 中 x/y 的结果不是int，故需转换
        forward, backward = int((len(s) / 2) - 1), int(len(s) / 2)
    else:
        forward, backward = int(len(s) / 2), int(len(s) / 2)
    while backward < len(s):
        if s[forward] != s[backward]:
            return False
        backward += 1
        forward -= 1
    return True


if __name__ == "__main__":
    s1 = 'abba'
    s2 = 'a'
    s3 = 'aca'
    s4 = 'aacc'
    print(is_palindrome1(s1))
    print(is_palindrome1(s2))
    print(is_palindrome1(s3))
    print(is_palindrome1(s4))
    print('2222222222222222')
    print(is_palindrome2(s1))
    print(is_palindrome2(s2))
    print(is_palindrome2(s3))
    print(is_palindrome2(s4))
    try:
        print(is_palindrome1(2))
    except ValueError as e:
        print(e)
