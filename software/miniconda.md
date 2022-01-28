# Conda, Poetry

[toc]

## 下载

[清华大学镜像站](https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/)
[conda](https://docs.conda.io/en/latest/miniconda.html)

下载一个 `Miniconda3-xxxx.sh`

## 安装

安装至指定目录

```bash
sudo sh Miniconda3-py38_4.8.2-Linux-x86_64.sh -f -b -p /opt/Miniconda3
```

修改 user group

```bash
sudo chown -R user:user /home/opt/Miniconda3
```

init

```bash
cd /opt/Miniconda3
bin/conda init [shell]
```

## 配置

`~/.condarc`

```text
channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
  - conda-forge
show_channel_urls: true
ssl_verify: true
changeps1: false
channel_priority: strict
```

`~/.pip/pip.conf`

```text
[global]
  index-url = https://pypi.tuna.tsinghua.edu.cn/simple
[install]
  trusted-host = pypi.tuna.tsinghua.edu.cn

[global]
  index-url = https://mirrors.aliyun.com/pypi/simple/
[install]
  trusted-host = mirrors.aliyun.com
```

## conda pip 命令

```bash
conda update --all
conda install package_name
conda create -n env_name python=3.8
conda activate env_name
pip install package_name
```

## poetry

`pyproject.toml`

```toml
[[tool.poetry.source]]

name = "aliyun"
url = "https://mirrors.aliyun.com/pypi/simple"
```
