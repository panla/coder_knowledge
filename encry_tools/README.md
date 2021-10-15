# README

利用 Cython 把 Python 项目/代码 转为 Linux 动态链接库，实现保护源码以及部分性能提升

## 为何利用 shell 来执行 python

参见 <https://github.com/panla/cython_build_demo>

## env

安装 jq

Debian/Ubuntu/Deepin

```bash
apt install jq
```

RHEL/CentOS

```bash
dnf install jq
yum install jq
```

## command

```bash
chmod 777 conf.json

sh ./start.sh ./conf.json
```

## conf.json

- `ORIGIN_DIR` 源文件夹路径
- `TARGET_DIR` 目标文件夹路径，会从 `ORIGIN_DIR` 复制到 `TARGET_DIR`
- `TRANSFORM_SCRIPT_PATH` 执行翻译任务的脚本路径
- `RENAME_SCRIPT_PATH` 执行重命名 so 文件的脚本路径
- `NAME_LIS` 被处理文件，文件夹定义
  - 只写需要处理的文件夹或文件
  - 当一个文件夹下的文件只剩一个 `__init__.py` 时，不写这个文件夹
  - 当一个文件夹下的子文件夹有需要处理的 py 文件时，需要写这个子文件夹的相对于根目录的相对路径

## 注意

需要用相同的Python版本来执行

## TODO

- [ ] 在 start.sh 中定义路径拼接
- [ ] 在 `transform_py_so.py` 中优化忽略文件
- [ ] 代码封装，打包
- [ ] 代码风格优化

## 方案参考致谢

- <https://github.com/Boris-code/jmpy>
