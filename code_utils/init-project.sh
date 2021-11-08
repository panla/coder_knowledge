#!/bin/sh

# usage，sh ./init-project.sh project_name


if [ ! -d "$1" ]; then
    mkdir $1
    echo "$1 had been created"
else
    echo "$1 had been exists"
fi


cd $1


# apps
mkdir apps
mkdir apps/libs -p
touch apps/libs/__ini__.py
touch apps/libs/init_app.py
mkdir apps/models -p
touch apps/models/__ini__.py
touch apps/models/model.py
mkdir apps/modules -p
touch apps/modules/__ini__.py
touch apps/__init__.py
touch apps/application.py


# common
mkdir common
touch common/__ini__.py


# conf
mkdir conf
touch conf/__ini__.py
touch conf/settings.py


# extensions
mkdir extensions
touch extensions/__ini__.py
touch extensions/define.py
touch extensions/response.py
touch extensions/exception.py


# redis_ext
mkdir redis_ext
touch redis_ext/__ini__.py
touch redis_ext/base.py


# scripts
mkdir scripts
touch scripts/__ini__.py


# services
mkdir services
touch services/__ini__.py


# sockets
mkdir sockets
touch sockets/__ini__.py
touch sockets/server.py
touch sockets/namesapce.py


# tests
mkdir tests
touch tests/__init__.py
touch tests/conftest.py
mkdir tests/fixture_data


# tools
mkdir tools
touch tools/__ini__.py


# docs
mkdir docs
mkdir docs/deploy -p
touch docs/deploy/docker-compose.yml
touch docs/deploy/config.py
touch docs/deploy/my.cnf
touch docs/deploy/nginx.conf
touch docs/deploy/docker-entrpoint.sh
touch docs/deploy/gunicorn_conf.py

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


# other
mkdir tmp logs

touch .gitignore README.md CHANGELOG.md server.py pytest.ini aerich.ini pyproject.toml
touch build.txt Dockerfile Makefile

cat>.gitignore<<EOF

/.idea/
/.vscode/

/tmp/
/logs/
*/__pycache__

/conf/project.local.toml
/conf/test.local.toml

*.sqlite
*.pyc

EOF

cat>pytest.ini<<EOF
[pytest]
filterwarnings =
    ignore::DeprecationWarning
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


echo "over"
