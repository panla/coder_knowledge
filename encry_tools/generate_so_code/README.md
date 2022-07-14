# README

[toc]

- 目标：实现Python项目源码加密保护，提升部分性能
- 实现: 利用 Cython 把 Python 代码转为 Linux 动态链接库

## command

```bash
python execute.py build -f [--config_file] ./config.json
```

## config 参数

- `origin_dir`: 源文件夹路径
- `target_dir`: 目标文件夹路径，会从 `ORIGIN_DIR` 复制到 `TARGET_DIR`
- `rules`: 相关规则

## 注意

在 A 环境 build 在 B 环境 run, 则需两个环境Python版本一致

目前只能处理相对简单的代码

## TODO

- [X] 从 distutils 转移至 setuptools
- [ ] 代码封装，打包
- [ ] 代码风格优化
- [ ] 优化适配忽略等规则

## 编译器选项

<https://cython.readthedocs.io/en/latest/src/userguide/source_files_and_compilation.html#compiler-directives>

## 参考方案致谢

- <https://github.com/Boris-code/jmpy>
