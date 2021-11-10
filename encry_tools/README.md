# README

- 目标：实现Python项目源码加密保护，提升部分性能
- 实现: 利用 Cython 把 Python 代码转为 Linux 动态链接库

## 为何利用 shell 来执行 python

参见 <https://github.com/panla/cython_build_demo>

## command

```bash
sh ./start.sh 源文件夹 目标文件夹 被处理文件的记录
sh ./start.sh ~/srv/solar_iter/solar_iter_api ~/tmp/py_so ./build.txt
```

## 参数

- `ORIGIN_DIR`: 源文件夹路径
- `TARGET_DIR`: 目标文件夹路径，会从 `ORIGIN_DIR` 复制到 `TARGET_DIR`
- `被处理文件的记录的文件路径`: 被处理文件，文件夹 相对于根路径的相对路径 定义的文件的路径
  - 只写需要处理的文件夹或文件
  - 当一个文件夹下的 py 文件只剩一个 `__init__.py` 时，不写这个文件夹
  - 当一个文件夹下的子文件夹有需要处理的 py 文件时，需要写这个子文件夹的相对于根目录的相对路径
  - 被处理的文件夹里有需要处理的py文件时，需要有一个`__init__.py` 即需要是一个模块/包

## 注意

在 A 环境 build 在 B 环境 run, 则需两个环境Python版本一致

## TODO

- [X] 在 start.sh 中定义路径拼接
- [ ] 在 `transform_py_so.py` 中优化忽略文件
- [ ] 代码封装，打包
- [ ] 代码风格优化
- [ ] 从 distutils 转移至 setuptools

## 编译器选项

<https://cython.readthedocs.io/en/latest/src/userguide/source_files_and_compilation.html#compiler-directives>

## 参考方案致谢

- <https://github.com/Boris-code/jmpy>
