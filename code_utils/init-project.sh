#!/bin/sh

# usage，sh ./init-project.sh project_name


if [ ! -d "$1" ]; then
    mkdir $1
    echo "$1 had been created"
else
    echo "$1 had been exists"
fi

echo "start"

cd $1


# apps
mkdir apps
mkdir apps/libs -p
touch apps/libs/__init__.py
touch apps/libs/init_app.py
mkdir apps/models -p
touch apps/models/__init__.py
touch apps/models/model.py
mkdir apps/modules -p
touch apps/modules/__init__.py
touch apps/__init__.py
touch apps/application.py

cat>apps/application.py<<EOF

def create_app():
    app = None

    return app
EOF


# common
mkdir common
touch common/__init__.py


# conf
mkdir conf
touch conf/__init__.py
touch conf/settings.py
touch conf/product.toml
touch conf/test.toml
touch conf/product.local.toml
touch conf/test.local.toml


# extensions
mkdir extensions
touch extensions/__init__.py
touch extensions/define.py
touch extensions/response.py
touch extensions/exception.py


# redis_ext
mkdir redis_ext
touch redis_ext/__init__.py
touch redis_ext/base.py
touch redis_ext/lock.py
touch redis_ext/sms.py


# scripts
mkdir scripts
touch scripts/__init__.py


# services
mkdir services
touch services/__init__.py


# sockets
mkdir sockets
touch sockets/__init__.py
touch sockets/server.py
touch sockets/namespace.py


# mixins
mkdir mixins
touch mixins/__init__.py
touch mixins/schema.py


# tests
mkdir tests
touch tests/__init__.py
touch tests/conftest.py
touch tests/utils.py
mkdir tests/fixture_data


# tools
mkdir tools
touch tools/__init__.py


# mirrors
mkdir mirrors
touch mirrors/sources.list
touch mirrors/requirements.txt
touch mirrors/requirements-simple.txt
touch mirrors/requirements-dev.txt

cat>mirrors/sources.list<<EOF
deb http://mirrors.aliyun.com/debian/ buster main non-free contrib
deb http://mirrors.aliyun.com/debian-security buster/updates main
deb http://mirrors.aliyun.com/debian/ buster-updates main non-free contrib
deb http://mirrors.aliyun.com/debian/ buster-backports main non-free contrib
EOF


# docs
mkdir docs
mkdir docs/deploy -p
touch docs/deploy/docker-compose.yml
touch docs/deploy/docker-entrpoint.sh
touch docs/deploy/gunicorn_conf.py
touch docs/deploy/my.cnf
touch docs/deploy/nginx.conf

cat>docs/deploy/my.cnf<<EOF
[mysqld]
port = 3306
mysqlx_port = 33060
bind-address = 0.0.0.0
default-authentication-plugin=mysql_native_password
character-set-server = utf8mb4
collation-server = utf8mb4_general_ci
init_connect='SET NAMES utf8mb4'

# wait_timeout = 3600

default-time-zone=+08:00
max_connections=2000
EOF


# other
mkdir tmp logs

touch .gitignore .dockerignore
touch README.md CHANGELOG.md Dockerfile Makefile docker-entrypoint.sh
touch config.py server.py
touch pytest.ini pyproject.toml build.txt

cat>.gitignore<<EOF

/.idea/
/.vscode/

/tmp/
/logs/
*/__pycache__

/conf/product.local.toml
/conf/test.local.toml

*.sqlite
*.pyc
EOF

cat>.dockerignore<<EOF
.git
.vscode
.idea
.svn

venv
.venv

tmp
logs
docs

**/__pycache__
EOF

cat>CHANGELOG.md<<EOF
# ChangeLog

EOF

cat>config.py<<EOF
import os
from pathlib import Path
from functools import lru_cache

import pytomlpp
from pydantic import BaseModel


BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class Setting(BaseModel):
    pass


@lru_cache()
def get_settings() -> Setting:
    CODE_ENV = os.environ.get('CODE_ENV', 'prd')

    if CODE_ENV == 'test':
        p = Path(BASE_DIR).joinpath('conf/test.local.toml')
    else:
        p = Path(BASE_DIR).joinpath('conf/product.local.toml')

    if not p.is_file():
        raise Exception('config no exists')

    settings = Setting.parse_obj(pytomlpp.load(p))
    return settings


Config = get_settings()
EOF

cat>server.py<<EOF
from apps.application import create_app

app = create_app()
EOF

cat>pytest.ini<<EOF
[pytest]
filterwarnings =
    ignore::DeprecationWarning
EOF

cat>pyproject.toml<<EOF
[tool.aerich]
tortoise_orm = "config.ORM_MIGRATE_CONF"
location = "./migrations"
src_folder = "./."
EOF

cat>build.txt<<EOF
apps/libs
apps/models
apps/modules
apps
common
conf
extensions
redis_ext
scripts
services
sockets
tests
EOF

cat>Dockerfile<<EOF
FROM python:3.8-slim-buster

ENV TZ=Asia/Shanghai LANG=C.UTF-8

VOLUME ["/home/project", "/home/logs"]

WORKDIR /home/project

EXPOSE 8000

COPY ./mirrors /mirrors

RUN cp /mirrors/sources.list /etc/apt/sources.list \ 
&& apt update && apt upgrade -y && apt autoclean -y && apt autoremove -y \ 
&& apt install gcc -y && python -m pip install --upgrade pip -i https://mirrors.aliyun.com/pypi/simple/ \ 
&& pip3 install -r /mirrors/requirements.txt -i https://mirrors.aliyun.com/pypi/simple/ --no-cache-dir

EOF

echo "over"
