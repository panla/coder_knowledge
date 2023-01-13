# README

从 c++ 生成动态链接库供 Python 调用

## 操作

```bash
mkdir -p build && cd build && cmake .. && make -j4
```

## 结果

```text
在 lib 下生成 .so

在 bin 下生成 main 可执行文件

cd tests

python3 1.py
产生打印
```
