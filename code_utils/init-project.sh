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
touch apps/libs/init.py
touch apps/libs/database.py
touch apps/libs/middleware.py
touch apps/libs/exception.py
mkdir apps/models -p
touch apps/models/__init__.py
touch apps/models/models.py
touch apps/models/fields.py
touch apps/models/mixin.py
mkdir apps/modules -p
touch apps/modules/__init__.py
touch apps/modules/resource.py
touch apps/modules/token.py
touch apps/__init__.py
touch apps/application.py
mkdir apps/test_api
touch apps/test_api/__init__.py

cat>apps/application.py<<EOF

def create_app():
    app = None

    return app
EOF

cat>apps/libs/database.py<<EOF
from fastapi import FastAPI
from tortoise.contrib.fastapi import register_tortoise

from config import ORM_LINK_CONF


def init_db(app: FastAPI):
    """init and bind tortoise-orm"""

    register_tortoise(app, config=ORM_LINK_CONF)
EOF

cat>apps/libs/exception.py<<EOF
import traceback
from typing import Any

from fastapi import FastAPI, Request, status
from fastapi.exceptions import HTTPException, RequestValidationError
from fastapi.responses import JSONResponse
from tortoise.validators import ValidationError

from conf.const import StatusCode
from extensions.log import logger
from extensions.exceptions import BaseHTTPException


def log_message(method: str, url: str, message: Any):
    """log message when catch exception"""

    logger.error('start error, this is'.center(60, '*'))
    logger.error(f'{method} {url}')
    logger.error(message)
    logger.error('end error'.center(60, '*'))


def register_exception(app: FastAPI):
    @app.exception_handler(BaseHTTPException)
    async def catch_c_http_exception(request: Request, exc: BaseHTTPException):
        """catch custom exception"""

        log_message(request.method, request.url, exc.message)
        content = exc.response()
        return JSONResponse(content=content, status_code=exc.status_code, headers=exc.headers)

    @app.exception_handler(HTTPException)
    async def http_exception_handler(request: Request, exc: HTTPException):
        """catch FastAPI HTTPException"""

        log_message(request.method, request.url, exc.detail)
        content = {'code': StatusCode.bad_request, 'message': exc.detail, 'data': None}
        return JSONResponse(content=content, status_code=exc.status_code, headers=exc.headers)

    @app.exception_handler(AssertionError)
    async def assert_exception_handle(request: Request, exc: AssertionError):
        """catch Python AssertError"""

        exc_str = ' '.join(exc.args)
        log_message(request.method, request.url, exc_str)
        content = {'code': StatusCode.validator_error, 'message': exc_str, 'data': None}
        return JSONResponse(content=content, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)

    @app.exception_handler(ValidationError)
    async def db_validation_exception_handle(request: Request, exc: ValidationError):
        """catch tortoise-orm ValidatorError"""

        exc_str = '|'.join(exc.args)
        log_message(request.method, request.url, exc.args)
        content = {'code': StatusCode.validator_error, 'message': exc_str, 'data': None}
        return JSONResponse(content=content, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        """catch FastAPI RequestValidationError"""

        exc_str = f'{exc}'.replace('\n', ' ').replace('   ', ' ')
        log_message(request.method, request.url, exc)
        # content = exc.errors()
        content = {'code': StatusCode.validator_error, 'message': exc_str, 'data': None}
        return JSONResponse(content=content, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)

    @app.exception_handler(Exception)
    async def exception_handle(request: Request, exc: Exception):
        """catch other exception"""

        log_message(request.method, request.url, traceback.format_exc())
        content = {'code': StatusCode.server_error, 'message': str(exc), 'data': None}
        return JSONResponse(content=content, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)
EOF

cat>apps/libs/middleware.py<<EOF
import time

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from conf.const import StatusCode

MiddlewareCodeContents = {
    405: {'code': StatusCode.method_not_allowed, 'message': 'METHOD_NOT_ALLOWED', 'data': None},
    406: {'code': StatusCode.not_acceptable, 'message': 'NOT_ACCEPTABLE', 'data': None},
    408: {'code': StatusCode.request_timeout, 'message': 'REQUEST_TIMEOUT', 'data': None},
    411: {'code': StatusCode.length_required, 'message': 'LENGTH_REQUIRED', 'data': None},
    413: {'code': StatusCode.entity_too_large, 'message': 'REQUEST_ENTITY_TOO_LARGE', 'data': None},
    414: {'code': StatusCode.request_uri_too_long, 'message': 'REQUEST_URI_TOO_LONG', 'data': None},
    431: {
        'code': StatusCode.header_fields_too_large,
        'message': 'REQUEST_HEADER_FIELDS_TOO_LARGE',
        'data': None
    }
}


def register_cross(app: FastAPI):
    """deal Cross Origin Resource Sharing"""

    app.add_middleware(
        CORSMiddleware,
        allow_methods=['*'],
        allow_headers=['*'],
        allow_credentials=True,
        allow_origin_regex='https?://.*',
        expose_headers=['X-TOKEN', 'X-Process-Time']
    )


def register_middleware(app: FastAPI):
    @app.middleware("http")
    async def rewrite_other_exception_response(request: Request, call_next):
        """overwrite response"""

        start_time = time.time()
        response = await call_next(request)

        m_content = MiddlewareCodeContents.get(response.status_code, None)
        if m_content:
            return JSONResponse(content=m_content, status_code=response.status_code)

        response.headers['X-Process-Time'] = str((time.time() - start_time) * 1000)
        return response
EOF

cat>apps/models/mixin.py<<EOF
from typing import Tuple, Type

from tortoise import fields
from tortoise.models import Model


class BaseModel(Model):
    id = fields.BigIntField(pk=True, description='主键')
    created_at = fields.DatetimeField(auto_now_add=True, null=False, description='创建时间')
    updated_at = fields.DatetimeField(auto_now=True, null=False, description='更新时间')
    is_deleted = fields.BooleanField(null=False, default=False, description='删除标识')

    @property
    def created_time(self) -> str:
        created_at = getattr(self, 'created_at')
        return created_at.strftime('%Y-%m-%d %H:%M:%S') if created_at else ''

    @property
    def updated_time(self) -> str:
        updated_at = getattr(self, 'updated_at')
        return updated_at.strftime('%Y-%m-%d %H:%M:%S') if updated_at else ''

    class Meta:
        abstract = True


class ModelMixin(object):
    __slots__ = ()

    def __init__(self, **kwargs):
        pass

    @staticmethod
    def to_dict(instance, selects: tuple = None, excludes: tuple = None) -> dict:

        if not hasattr(instance, '_meta'):
            raise AssertionError('<%r> does not have attribute for _meta' % instance)

        if selects:
            return {i: getattr(instance, i) for i in selects}
        elif excludes:
            return {i: getattr(instance, i) for i in instance._meta.fields if i not in excludes}
        else:
            return {i: getattr(instance, i) for i in instance._meta.fields}

    async def async_to_dict(self, selects: tuple = None, excludes: tuple = None, second_attrs: dict = None) -> dict:
        """response dict data of instance serialize

        selects: ('id', 'name')
        excludes: ('created_at', 'updated_at')
        second_attrs: {'owner': ['id', 'name'], 'agency': ['id', 'name']}
        """

        results = self.to_dict(self, selects=selects, excludes=excludes)
        if second_attrs:
            for attr, fields in second_attrs.items():
                results.update({attr: self.to_dict(await getattr(self, attr), selects=fields)})

        return results

    def sync_to_dict(self, selects: tuple = None, excludes: tuple = None, second_attrs: dict = None) -> dict:
        """response dict data of instance serialize

        selects: ('id', 'name')
        excludes: ('created_at', 'updated_at')
        second_attrs: {'owner': ['id', 'name'], 'agency': ['id', 'name']}
        """

        results = self.to_dict(self, selects=selects, excludes=excludes)
        if second_attrs:
            for attr, fields in second_attrs.items():
                results.update({attr: self.to_dict(getattr(self, attr), selects=fields)})

        return results
EOF

cat>apps/modules/resource.py<<EOF
from extensions import NotFound


class ResourceOp():
    def __init__(self, model, pk):
        self.model = model
        self.pk = pk

    async def instance(self, is_delete=None):
        if is_delete is not None:
            _instance = await self.model.filter(id=self.pk, is_delete=is_delete).first()
        else:
            _instance = await self.model.filter(id=self.pk).first()
        if not _instance:
            raise NotFound(message=f'Model = {self.model.__name__}, pk = {self.pk} is not exists')
        return _instance
EOF


# common
mkdir common
touch common/__init__.py
touch common/tools.py


# conf
mkdir conf
touch conf/__init__.py
touch conf/settings.py
touch conf/const.py
touch conf/product.toml
touch conf/test.toml
touch conf/product.local.toml
touch conf/test.local.toml

cat>conf/const.py<<EOF
class StatusCode(object):
    success = 10000

    bad_request = 40000
    unauthorized = 40100
    forbidden = 40300
    not_found = 40400
    method_not_allowed = 40500
    not_acceptable = 40600
    request_timeout = 40800
    length_required = 41100
    entity_too_large = 41300
    request_uri_too_long = 41400
    validator_error = 42200
    locked = 42300
    header_fields_too_large = 43100

    server_error = 45000
    unknown_error = 45001


class PaginateConst:
    DefaultNum = 1
    DefaultSize = 10

    MinNum = 1
    MaxSize = 40
EOF


# extensions
mkdir extensions
touch extensions/__init__.py
touch extensions/log.py
touch extensions/route.py
touch extensions/schema.py
touch extensions/exceptions.py
touch extensions/paginate.py
touch extensions/response.py

cat>extensions/log.py<<EOF
__all__ = ['logger']

import sys
from pathlib import Path

from loguru import logger

from config import LogConfig

LOG_LEVEL = LogConfig.LOG_LEVEL
LOG_PATH = LogConfig.LOG_PATH

Path(LOG_PATH).parent.mkdir(parents=True, exist_ok=True)

logger.remove()

logger.add(
    LOG_PATH, level=LOG_LEVEL.upper(), rotation="00:00", backtrace=True, diagnose=True, enqueue=True,
)
logger.add(sys.stdout, level=LOG_LEVEL.upper(), backtrace=True, diagnose=True, enqueue=True)
EOF

cat>extensions/route.py<<EOF
from json import JSONDecodeError

from fastapi import Request
from fastapi.routing import APIRoute

from extensions import logger


class Route(APIRoute):
    """Extension Route and log request.json()"""

    def get_route_handler(self):
        original_route_handler = super().get_route_handler()

        async def log_request_detail(request: Request):

            logger.info('start request'.center(60, '*'))
            logger.info(f'{request.method} {request.url}')

            methods = ['POST', 'PUT', 'PATCH']
            content_type = request.headers.get('content-type')

            if request.method in methods and 'application/json' in content_type:
                try:
                    params = await request.json()
                    if params:
                        logger.info(params)
                except JSONDecodeError:
                    logger.error('encounter JSONDecodeError')
                except UnicodeDecodeError:
                    logger.error('encounter UnicodeDecodeError')
            logger.info('end request'.center(60, '*'))
            return await original_route_handler(request)

        return log_request_detail
EOF

cat>extensions/schema.py<<EOF
from typing import Optional, Any

from fastapi import Query
from pydantic import BaseModel
from pydantic.typing import NoneType

from conf.const import StatusCode, PaginateConst


class BadRequestSchema(BaseModel):
    code: int = StatusCode.bad_request
    message: str = ''
    data: NoneType = "null"


class UnauthorizedSchema(BaseModel):
    code: int = StatusCode.unauthorized
    message: str = ''
    data: NoneType = "null"


class ForbiddenSchema(BaseModel):
    code: int = StatusCode.forbidden
    message: str = ''
    data: NoneType = "null"


class NotFoundSchema(BaseModel):
    code: int = StatusCode.not_found
    message: str = ''
    data: NoneType = "null"


class ValidatorErrorSchema(BaseModel):
    code: int = StatusCode.validator_error
    message: str = ''
    data: NoneType = "null"


ErrorSchema = {
    400: {
        'model': BadRequestSchema,
        'description': 'bad_request'
    },
    401: {
        'model': UnauthorizedSchema,
        'description': 'unauthorized'
    },
    403: {
        'model': ForbiddenSchema,
        'description': 'forbidden'
    },
    404: {
        'model': NotFoundSchema,
        'description': 'not_found'
    },
    422: {
        'model': ValidatorErrorSchema,
        'description': 'request parameters validator'
    }
}


class SchemaMixin(BaseModel):
    code: int = 10000
    message: str = ''
    data: Optional[Any]


class NormalSchema(SchemaMixin):
    """"""

    data: Optional[str] = 'success'


class FilterParserMixin(BaseModel):
    """search list data"""

    page: Optional[int] = Query(PaginateConst.DefaultNum, title='page', gte=PaginateConst.MinNum)
    pagesize: Optional[int] = Query(PaginateConst.DefaultSize, title='pagesize', gte=1, lte=PaginateConst.MaxSize)
EOF

cat>extensions/exceptions.py<<EOF
from typing import Optional, Any, Dict

from fastapi import status
from starlette.exceptions import HTTPException

from conf.const import StatusCode
from .schema import SchemaMixin


class BaseHTTPException(HTTPException):
    MESSAGE = None
    STATUS_CODE = status.HTTP_400_BAD_REQUEST
    CODE = 40000

    def __init__(
            self,
            message: Any = None,
            code: int = None,
            headers: Optional[Dict[str, Any]] = None
    ) -> None:
        self.message = message or self.MESSAGE
        self.status_code = self.STATUS_CODE
        self.code = code or self.CODE
        self.detail = self.message
        self.headers = headers

    def __repr__(self) -> str:
        class_name = self.__class__.__name__
        return f"{class_name}(status_code={self.status_code!r}, code={self.code}, msg={self.message!r})"

    def response(self):
        return SchemaMixin(code=self.code, message=self.message, data=None).dict()


class BadRequest(BaseHTTPException):
    STATUS_CODE = status.HTTP_400_BAD_REQUEST
    CODE = StatusCode.bad_request


class Unauthorized(BaseHTTPException):
    STATUS_CODE = status.HTTP_401_UNAUTHORIZED
    CODE = StatusCode.unauthorized


class Forbidden(BaseHTTPException):
    STATUS_CODE = status.HTTP_403_FORBIDDEN
    CODE = StatusCode.forbidden


class NotFound(BaseHTTPException):
    STATUS_CODE = status.HTTP_404_NOT_FOUND
    CODE = StatusCode.not_found


class MethodNotAllowed(BaseHTTPException):
    STATUS_CODE = status.HTTP_405_METHOD_NOT_ALLOWED
    CODE = StatusCode.method_not_allowed


class Locked(BaseHTTPException):
    STATUS_CODE = status.HTTP_423_LOCKED
    CODE = StatusCode.locked
EOF

cat>extensions/paginate.py<<EOF
from tortoise.queryset import QuerySet, MODEL

from conf.const import PaginateConst


class Pagination(object):
    def __init__(
            self,
            query: QuerySet,
            page: int = PaginateConst.DefaultNum,
            pagesize: int = PaginateConst.DefaultSize
    ) -> None:
        self.query = query
        self.page = page
        self.pagesize = pagesize

    def items(self) -> QuerySet[MODEL]:
        return self.query.offset((self.page - 1) * self.pagesize).limit(self.pagesize)
EOF

cat>extensions/response.py<<EOF
from typing import Any

from extensions import logger


def resp_success(message: str = '', print_msg: str = '', data: Any = None):
    if print_msg:
        pass
    else:
        if message and message != 'success':
            print_msg = message

    if print_msg:
        logger.info(print_msg)

    return {'message': message, 'data': data}
EOF


# redis_ext
mkdir redis_ext
touch redis_ext/__init__.py
touch redis_ext/base.py
touch redis_ext/lock.py
touch redis_ext/sms.py

cat>redis_ext/base.py<<EOF
from typing import Union
from datetime import timedelta

from aioredis import Redis

from config import RedisConfig


class BaseRedisClient(object):
    DB = 0
    PREFIX_KEY = ''
    CONNECTION_PARAMS = {'encoding': 'utf-8', 'decode_responses': True}

    def __init__(self, key) -> None:
        self.key = f'{self.PREFIX_KEY}:{key}'
        self.uri = 'redis://:{}@{}:{}/{}'.format(
            RedisConfig.REDIS_PASSWD, RedisConfig.REDIS_HOST, RedisConfig.REDIS_PORT, self.DB
        )

    @property
    def client(self) -> Redis:
        # source code
        # connection_pool = ConnectionPool.from_url(url, **kwargs)

        return Redis.from_url(self.uri, **self.CONNECTION_PARAMS)
EOF


# scripts
mkdir scripts
mkdir scripts/init_data
touch scripts/insert.py
touch scripts/__init__.py


# services
mkdir services
touch services/__init__.py


# tests
mkdir tests
touch tests/__init__.py
touch tests/conftest.py
touch tests/utils.py
mkdir tests/fixture_data


# tools
mkdir tools
touch tools/__init__.py
touch tools/worker.py

cat>tools/worker.py<<EOF
"""
extend gunicorn worker_class, if had installed uvloop httptols, use them
"""

from uvicorn.workers import UvicornWorker

try:
    import uvloop
    import httptools


    class Worker(UvicornWorker):
        CONFIG_KWARGS = {"loop": "uvloop", "http": "httptools"}

    del httptools
    del uvloop
except ImportError:
    class Worker(UvicornWorker):
        CONFIG_KWARGS = {"loop": "asyncio", "http": "auto"}
EOF


# mirrors
mkdir mirrors
touch mirrors/README.md
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
touch pytest.ini pyproject.toml

cat>.gitignore<<EOF

/.idea/
/.vscode/

/tmp/
/logs/
*/__pycache__

/conf/product.local.toml
/conf/test.local.toml
/docker-entrypoint.sh

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

*.pyc
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
    code_env = os.environ.get('CODE_ENV', 'prd')

    if code_env == 'test':
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
