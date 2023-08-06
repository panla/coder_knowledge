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
mkdir -p apps
mkdir -p apps/libs
touch apps/libs/__init__.py
touch apps/libs/init.py
touch apps/libs/database.py
touch apps/libs/middleware.py
touch apps/libs/exception.py
touch apps/libs/app_events.py

cat>apps/libs/__init__.py<<EOF
from .exception import register_exception
from .middleware import register_cross, register_middleware
from .init import init_app
EOF
