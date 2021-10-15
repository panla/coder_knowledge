# README

利用 Cython 把 Python 项目/代码 转为 Linux 动态链接库，实现保护源码以及部分性能提升

## 为何利用 shell 来执行 python

参见 <https://github.com/panla/cython_build_demo>

## command

```bash
sh ./start.sh 原文件夹 目标文件夹 被处理文件的记录
sh ./start.sh ~/srv/solar_iter/solar_iter_api ~/tmp/py_so ./build.txt
```

## 参数

- `ORIGIN_DIR` 源文件夹路径
- `TARGET_DIR` 目标文件夹路径，会从 `ORIGIN_DIR` 复制到 `TARGET_DIR`
- `被处理文件的记录` 被处理文件，文件夹定义的文件路径
  - 只写需要处理的文件夹或文件
  - 当一个文件夹下的文件只剩一个 `__init__.py` 时，不写这个文件夹
  - 当一个文件夹下的子文件夹有需要处理的 py 文件时，需要写这个子文件夹的相对于根目录的相对路径
  - 被处理的文件夹里有需要处理的py文件时，需要有一个`__init__.py`

## 注意

需要用相同的Python版本来执行

## TODO

- [ ] 在 start.sh 中定义路径拼接
- [ ] 在 `transform_py_so.py` 中优化忽略文件
- [ ] 代码封装，打包
- [ ] 代码风格优化

## 编译器选项

<https://cython.readthedocs.io/en/latest/src/userguide/source_files_and_compilation.html#compiler-directives>

## 方案参考致谢

- <https://github.com/Boris-code/jmpy>
