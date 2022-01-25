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
mkdir apps/v1_api
touch apps/v1_api/__init__.py
mkdir apps/v1_api/entities
mkdir apps/v1_api/logics
mkdir apps/v1_api/resources
touch apps/v1_api/entities/__init__.py
touch apps/v1_api/logics/__init__.py
touch apps/v1_api/resources/__init__.py

cat>apps/libs/__init__.py<<EOF
from .exception import register_exception
from .middleware import register_cross, register_middleware
from .init import init_app
EOF

cat>apps/v1_api/__init__.py<<EOF
from fastapi import FastAPI

from apps.libs.exception import register_exception
from .resources import user


def register_routers(app: FastAPI):
    """register routers"""

    app.include_router(user.router, prefix='/users', tags=['User'])


def init_sub_app(app: FastAPI):
    """mount sub app"""

    api_app: FastAPI = FastAPI()

    register_exception(api_app)
    register_routers(api_app)
    app.mount(path='/api/v1', app=api_app, name='v1_api')

    return app
EOF

cat>apps/modules/__init__.py<<EOF
from .resource import ResourceOp
EOF

cat>apps/application.py<<EOF
from fastapi import FastAPI

from apps.libs import init_app


def create_app() -> FastAPI:
    app = FastAPI()

    app = init_app(app)

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
from typing import Union, Any

from fastapi import FastAPI, Request, status
from fastapi.exceptions import HTTPException, RequestValidationError
from fastapi.responses import JSONResponse
from starlette.datastructures import URL
from tortoise.validators import ValidationError

from conf.const import StatusCode
from extensions.log import logger
from extensions.exceptions import BaseHTTPException


def log_message(method: str, url: Union[str, URL], message: Any):
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

cat>apps/libs/init.py<<EOF
from fastapi import FastAPI

from .database import init_db
from .middleware import register_cross, register_middleware


def init_app(app: FastAPI) -> FastAPI:

    init_db(app)
    register_cross(app)
    register_middleware(app)

    return app
EOF

cat>apps/models/fields.py<<EOF
"""
Num
    8-bit : TinyInt     UnsignedTinyInt
    16-bit: SmallInt    UnsignedSmallInt
    24-bit: MediumInt   UnsignedMediumInt
    32-bit: Int         UnsignedInt
    64-bit: BitInt      UnsignedBitInt

    FloatField: DOUBLE
    DecimalField

EnumField
    IntEnumField    0 <= value < 32768
    CharEnumField
"""
from typing import Any

from tortoise.fields.data import IntField, BigIntField


class TinyIntField(IntField):
    """
    Tiny integer field. (8-bit unsigned)

    pk (bool):
        True if field is Primary Key.
    """

    SQL_TYPE = "TINYINT"
    allows_generated = True

    def __init__(self, pk: bool = False, **kwargs: Any) -> None:
        if pk:
            kwargs["generated"] = bool(kwargs.get("generated", True))
        super().__init__(pk=pk, **kwargs)

    @property
    def constraints(self) -> dict:
        return {
            "ge": 1 if self.generated or self.reference else -128,
            "le": 127
        }

    class _db_mysql:
        GENERATED_SQL = "TINYINT NOT NULL PRIMARY KEY AUTO_INCREMENT"


class MediumIntField(IntField):
    """
    Medium integer field. (24-bit unsigned)

    pk (bool):
        True if field is Primary Key.
    """

    SQL_TYPE = "MEDIUMINT"
    allows_generated = True

    def __init__(self, pk: bool = False, **kwargs: Any) -> None:
        if pk:
            kwargs["generated"] = bool(kwargs.get("generated", True))
        super().__init__(pk=pk, **kwargs)

    @property
    def constraints(self) -> dict:
        return {
            "ge": 1 if self.generated or self.reference else -8388608,
            "le": 8388607
        }

    class _db_mysql:
        GENERATED_SQL = "MEDIUMINT NOT NULL PRIMARY KEY AUTO_INCREMENT"


class UnsignedTinyIntField(IntField):
    """
    Unsigned Tiny integer field. (8-bit unsigned)

    pk (bool):
        True if field is Primary Key.
    """

    SQL_TYPE = "TINYINT UNSIGNED"
    allows_generated = True

    def __init__(self, pk: bool = False, **kwargs: Any) -> None:
        if pk:
            kwargs["generated"] = bool(kwargs.get("generated", True))
        super().__init__(pk=pk, **kwargs)

    @property
    def constraints(self) -> dict:
        return {
            "ge": 1 if self.generated or self.reference else 0,
            "le": 255
        }

    class _db_mysql:
        GENERATED_SQL = "TINYINT UNSIGNED NOT NULL PRIMARY KEY AUTO_INCREMENT"


class UnsignedSmallIntField(IntField):
    """
    Unsigned Small integer field. (16-bit unsigned)

    pk (bool):
        True if field is Primary Key.
    """

    SQL_TYPE = "SMALLINT UNSIGNED"
    allows_generated = True

    def __init__(self, pk: bool = False, **kwargs: Any) -> None:
        if pk:
            kwargs["generated"] = bool(kwargs.get("generated", True))
        super().__init__(pk=pk, **kwargs)

    @property
    def constraints(self) -> dict:
        return {
            "ge": 1 if self.generated or self.reference else 0,
            "le": 65535
        }

    class _db_mysql:
        GENERATED_SQL = "SMALLINT UNSIGNED NOT NULL PRIMARY KEY AUTO_INCREMENT"


class UnsignedMediumIntField(IntField):
    """
    Unsigned Medium integer field. (24-bit unsigned)

    pk (bool):
        True if field is Primary Key.
    """

    SQL_TYPE = "MEDIUMINT UNSIGNED"
    allows_generated = True

    def __init__(self, pk: bool = False, **kwargs: Any) -> None:
        if pk:
            kwargs["generated"] = bool(kwargs.get("generated", True))
        super().__init__(pk=pk, **kwargs)

    @property
    def constraints(self) -> dict:
        return {
            "ge": 1 if self.generated or self.reference else 0,
            "le": 16777215
        }

    class _db_mysql:
        GENERATED_SQL = "MEDIUMINT UNSIGNED NOT NULL PRIMARY KEY AUTO_INCREMENT"


class UnsignedIntField(IntField):
    """
    Unsigned Int integer field. (32-bit unsigned)

    pk (bool):
        True if field is Primary Key.
    """

    SQL_TYPE = "INT UNSIGNED"
    allows_generated = True

    def __init__(self, pk: bool = False, **kwargs: Any) -> None:
        if pk:
            kwargs["generated"] = bool(kwargs.get("generated", True))
        super().__init__(pk=pk, **kwargs)

    @property
    def constraints(self) -> dict:
        return {
            "ge": 1 if self.generated or self.reference else 0,
            "le": 4294967295
        }

    class _db_mysql:
        GENERATED_SQL = "INT UNSIGNED NOT NULL PRIMARY KEY AUTO_INCREMENT"


class UnsignedBigIntField(BigIntField):
    """
    Unsigned Big integer field. (64-bit unsigned)

    pk (bool):
        True if field is Primary Key.
    """

    SQL_TYPE = "BIGINT UNSIGNED"
    allows_generated = True

    def __init__(self, pk: bool = False, **kwargs: Any) -> None:
        if pk:
            kwargs["generated"] = bool(kwargs.get("generated", True))
        super().__init__(pk=pk, **kwargs)

    @property
    def constraints(self) -> dict:
        return {
            "ge": 1 if self.generated or self.reference else 0,
            "le": 18446744073709551615
        }

    class _db_mysql:
        GENERATED_SQL = "BIGINT UNSIGNED NOT NULL PRIMARY KEY AUTO_INCREMENT"
EOF

cat>apps/models/mixin.py<<EOF
from tortoise import fields
from tortoise.models import Model

from .fields import UnsignedBigIntField


class BaseModel(Model):
    id = UnsignedBigIntField(pk=True, description='主键')
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
            for attr, columns in second_attrs.items():
                results.update({attr: self.to_dict(await getattr(self, attr), selects=columns)})

        return results

    def sync_to_dict(self, selects: tuple = None, excludes: tuple = None, second_attrs: dict = None) -> dict:
        """response dict data of instance serialize

        selects: ('id', 'name')
        excludes: ('created_at', 'updated_at')
        second_attrs: {'owner': ['id', 'name'], 'agency': ['id', 'name']}
        """

        results = self.to_dict(self, selects=selects, excludes=excludes)
        if second_attrs:
            for attr, columns in second_attrs.items():
                results.update({attr: self.to_dict(getattr(self, attr), selects=columns)})

        return results
EOF

cat>apps/modules/resource.py<<EOF
from extensions import NotFound


class ResourceOp():
    def __init__(self, model, pk):
        self.model = model
        self.pk = pk

    async def instance(self, is_deleted=None):
        _instances = self.model.filter(id=self.pk)
        if is_deleted is not None:
            _instance = _instances.filter(is_deleted=is_deleted)
        _instance = await _instances.first()
        if not _instance:
            raise NotFound(message=f'Model = {self.model.__name__}, pk = {self.pk} is not exists')
        return _instance
EOF


# commen
mkdir common
touch common/__init__.py
touch common/tools.py

cat>common/__init__.py<<EOF
from .tools import UidGenerator
EOF

cat>common/tools.py<<EOF
import uuid


class UidGenerator:

    def u_id(self) -> str:
        return f'{uuid.uuid4().hex}'

    def __str__(self) -> str:
        return self.u_id()

    def __repr__(self) -> str:
        return self.u_id()
EOF

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

cat>conf/settings.py<<EOF
from functools import lru_cache

from pydantic import BaseModel


class LogSetting(BaseModel):
    # 日志

    LOG_LEVEL: str = 'DEBUG'
    LOG_PATH: str


class DBSetting(BaseModel):
    # mysql

    DB_POOL_RECYCLE: str = 1000

    DB_USER: str = 'root'
    DB_PASSWD: str
    DB_HOST: str = '127.0.0.1'
    DB_PORT: int = 3306
    DB_DATABASE: str
    DB_MAX_SIZE: int = 5


class MQSetting(BaseModel):
    # rabbitmq

    HOST: str
    PORT: int
    USER: str
    PASSWD: str


class RedisSetting(BaseModel):
    # redis

    HOST: str
    PORT: int = 6379
    USER: str
    PASSWD: str


class MQTTSetting(BaseModel):
    # mqtt

    HOST: str
    PORT: int
    USER: str
    PASSWD: str
    KEEPALIVE: int = 600


class ORMSetting():
    def __init__(self, db):
        self.db = db

    def _base_orm_conf(self, apps: dict) -> dict:
        return {
            'connections': {
                'default': {
                    'engine': 'tortoise.backends.mysql',
                    'credentials': {
                        'host': self.db.DB_HOST,
                        'port': self.db.DB_PORT,
                        'user': self.db.DB_USER,
                        'password': self.db.DB_PASSWD,
                        'database': self.db.DB_DATABASE,
                        'minsize': 1,
                        'maxsize': self.db.DB_MAX_SIZE,
                        'charset': 'utf8mb4',
                        'pool_recycle': self.db.DB_POOL_RECYCLE
                    }
                }
            },
            'apps': apps,
            'use_tz': False,
            'timezone': 'Asia/Shanghai'
        }

    @property
    @lru_cache
    def orm_link_conf(self) -> dict:
        orm_apps_settings = {
            'models': {
                'models': [
                    'aerich.models',
                    'apps.models.models'
                ],
                'default_connection': 'default',
            }
        }
        return self._base_orm_conf(orm_apps_settings)

    @property
    def orm_migrate_conf(self) -> dict:
        orm_apps_settings = {
            'models': {
                'models': [
                    'aerich.models',
                    'apps.models.models'
                ],
                'default_connection': 'default',
            }
        }
        return self._base_orm_conf(orm_apps_settings)

    @property
    def orm_test_migrate_conf(self) -> dict:
        orm_apps_settings = {
            'models': {
                'models': [
                    'aerich.models',
                    'apps.models.models'
                ],
                'default_connection': 'default',
            }
        }
        return self._base_orm_conf(orm_apps_settings)
EOF

cat>conf/product.toml<<EOF
[log]
LOG_LEVEL = "INFO"
LOG_PATH = "/home/logs/x.log"

[db]
DB_POOL_RECYCLE = 1800

DB_USER = "root"
DB_PASSWD = "passwd"
DB_HOST = "127.0.0.1"
DB_PORT = 3306
DB_DATABASE = "database"
DB_MAX_SIZE = 500

[mq]
HOST = "127.0.0.1"
PORT = 5672
USER = "root"
PASSWD = "root"

[mqtt]
HOST = "127.0.0.1"
PORT = 1883
USER = "backend"
PASSWD = "backend"
KEEPALIVE = 600

[redis]
HOST = "127.0.0.1"
PORT = 6379
USER = "default"
PASSWD = "passwd"
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

cat>extensions/__init__.py<<EOF
from .log import logger
from .route import Route
from .schema import ErrorSchema, SchemaMixin, NormalSchema, FilterParserMixin
from .exceptions import (
    BaseHTTPException,
    BadRequest,
    Unauthorized,
    Forbidden,
    NotFound,
    MethodNotAllowed,
    Locked
)
from .paginate import Pagination
from .response import resp_success
EOF

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

from .log import logger


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
from aioredis import Redis

from config import RedisConfig


class BaseRedisClient(object):
    DB = 0
    PREFIX_KEY = ''
    CONNECTION_PARAMS = {'encoding': 'utf-8', 'decode_responses': True}

    def __init__(self) -> None:
        self._name = None
        self.uri = 'redis://{}:{}@{}:{}/{}'.format(
            RedisConfig.USER, RedisConfig.PASSWD, RedisConfig.HOST, RedisConfig.PORT, self.DB
        )
        self.client: Redis = Redis.from_url(self.uri, **self.CONNECTION_PARAMS)

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = f'{self.PREFIX_KEY}:{value}'
EOF


# scripts
mkdir scripts
mkdir scripts/init_data
touch scripts/insert.py
touch scripts/__init__.py

cat>scripts/insert.py<<EOF
import asyncio
import sys
import json
from pathlib import Path
from typing import List
from functools import wraps

import click
from tortoise import Tortoise
from tortoise.transactions import atomic
from loguru import logger

BASEDIR = Path(__file__).parent.parent

sys.path.append(BASEDIR.name)

from config import ORM_LINK_CONF


def _read_json_file(path: str) -> List[dict]:
    """read json file and return dict"""

    with open(path, 'r', encoding='utf-8') as f:
        return json.loads(f.read())


def coro(f):
    @wraps(f)
    def wrapper(*args, **kwargs):
        loop = asyncio.get_event_loop()
        try:
            loop.run_until_complete(f(*args, **kwargs))
        finally:
            if f.__name__ != "cli":
                loop.run_until_complete(Tortoise.close_connections())

    return wrapper


async def create(model, params: List[dict]):
    for param in params:
        instance = await model.get_or_none(id=param.get('id'))
        if instance:
            await instance.update_from_dict(param)
            await instance.save()
        else:
            instance = await model.create(**param)


@click.group()
@click.pass_context
@coro
async def cli(ctx: click.Context):
    await Tortoise.init(config=ORM_LINK_CONF)


@click.command(help='create init users data')
@click.pass_context
@click.option('-f', '--file', help='json file')
@coro
@atomic()
async def create_instances(ctx: click.Context, file: str):

    pass


cli.add_command(create_instances)

if __name__ == '__main__':
    cli()
EOF

# services
mkdir services
touch services/__init__.py
mkdir services/mqtt_services
mkdir services/celery_services
touch services/mqtt_services/__init__.py
touch services/celery_services/__init__.py
touch services/mqtt_services/client.py

cat>services/mqtt_services/client.py<<EOF
import json

from paho.mqtt.client import Client

from extensions.log import logger


class MqttClient:
    def __init__(self) -> None:
        self.client = Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish
        self.client.on_disconnect = self.on_disconnect
        self.client.on_unsubscribe = self.on_unsubscribe
        self.client.on_subscribe = self.on_subscribe

    def connect(self, host: str, port: int, user: str, passwd: str, keepalive: int = 600):
        """加载账户，连接"""

        self.client.username_pw_set(user, passwd)
        self.client.connect(host=host, port=port, keepalive=keepalive)

    def loop_start(self):
        """loop"""
        self.client.loop_start()

    def loop_forever(self):
        """循环保持"""

        self.client.loop_forever()

    def subscribe(self, topic: str):
        """订阅 topic"""

        self.client.subscribe(topic)

    def publish(self, topic, payload, qos=0):
        """发布"""

        data = json.dumps(payload, ensure_ascii=False)
        self.client.publish(topic=topic, payload=data, qos=qos)

    def add_callback(self, topic, callback):
        """向指定的 topic 添加回复/回调"""

        self.client.message_callback_add(topic, callback)

    def on_connect(self, client, userdata, flags, rc):
        """连接事件"""

        logger.info('on_connect'.center(40, '*'))
        logger.info(f'Connected with result code: {rc}')

    def on_message(self, client, userdata, msg):
        """获得消息事件，触发动作，匹配不到 message_callback_add 时使用这个"""

        logger.info('on_message'.center(40, '*'))
        payload = msg.payload.decode('utf-8')
        logger.info(f'on_message topic: {msg.topic}')
        logger.info(payload)

    def on_subscribe(self, client, userdata, mid, granted_qos):
        """订阅事件"""

        logger.info('on_subscribe'.center(40, '*'))
        logger.info('on_subscribe: qos = {granted_qos}')

    def on_unsubscribe(self, client, userdata, mid):
        """取消订阅事件"""

        logger.info('on_unsubscribe'.center(40, '*'))
        logger.info('on_unsubscribe: qos = {granted_qos}')

    def on_publish(self, client, userdata, mid):
        """发布消息事件"""

        logger.info('on_publish'.center(40, '*'))
        logger.info(f'on_publish: mid = {mid}')

    def on_disconnect(self, client, userdata, rc):
        """断开连接事件"""

        logger.info('on_disconnect'.center(40, '*'))
        logger.info('Unexpected disconnected rc = {rc}')
EOF

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
/gunicorn_config.py

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

from conf.settings import (
    LogSetting, DBSetting, MQTTSetting, ORMSetting, MQSetting, RedisSetting
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class Setting(BaseModel):
    log: LogSetting
    mqtt: MQTTSetting
    db: DBSetting
    mq: MQSetting
    redis: RedisSetting


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

ORM_LINK_CONF = ORMSetting(Config.db).orm_link_conf
ORM_MIGRATE_CONF = ORMSetting(Config.db).orm_migrate_conf
ORM_TEST_MIGRATE_CONF = ORMSetting(Config.db).orm_test_migrate_conf

LogConfig = Config.log
MQTTConfig = Config.mqtt
RedisConfig = Config.redis
MQConfig = Config.mq
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
