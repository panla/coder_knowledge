# struct 模块

[toc]

## doc

<https://docs.python.org/zh-cn/3/library/struct.html#format-characters>

## 函数

```text
struct.pack(format, v1, v2, ...)
    return bytes

struct.unpack(format, buffer)
    return tuple
```

## 字节顺序，大小，对齐方式

```text
大端，较低的有效字节存放在较高的存储器地址中，较高的有效字节存放在较低的存储器地址
小端，较高的有效字节存放在较高的存储器地址中，较低的有效字节存放在较低的存储器地址

    0x  12   34   56   78
        0x00 0x01 0x02 0x03
大端    12   34   56   78
小端    78   56   34   12

```

| 字符 | 字节顺序 | 大小 | 对齐方式 |
| :-: | :-: | :-: | :-: |
| @，默认，本机 | 按照原字节 | 按照原字节 | 按照原字节 |
| = | 按照原字节 | 标准 | 无 |
| < | 小端 | 标准 | 无 |
| > | 大端 | 标准 | 无 |
| ! | 网络(=大端) | 标准 | 无 |
