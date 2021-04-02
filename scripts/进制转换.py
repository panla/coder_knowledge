class Stack(object):
    def __init__(self):
      self.items = []

    def push(self, value):
        self.items.append(value)

    def pop(self):
        return self.items.pop()

    def is_empty(self):
        return not self.items


def to_str(num, base):
    """递归，10进制转其他进制"""

    convertstr = '0123456789ABCDEFG'
    if num < base:
        return convertstr[num]
    else:
        return to_str(num // base, base) + convertstr[num%base]


def to_str_2(num, base):
    """通过栈的方式转换进制"""

    stack = Stack()
    convertstr = '0123456789ABCDEFG'
    while num > 0:
        if num < base:
            stack.push(convertstr[num])
        else:
            stack.push(convertstr[num % base])
        num = num // base
    res = ''
    while not stack.is_empty():
        res += str(stack.pop())
    return res


print(to_str(1453, 16))
print(to_str_2(1453, 16))
