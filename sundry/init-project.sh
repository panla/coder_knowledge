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
touch apps/libs/app_events.py

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

mkdir apps/v1_api/schemas
mkdir apps/v1_api/logics
mkdir apps/v1_api/endpoints
touch apps/v1_api/schemas/__init__.py
touch apps/v1_api/logics/__init__.py
touch apps/v1_api/endpoints/__init__.py

cat>apps/libs/__init__.py<<EOF
from .exception import register_exception
from .middleware import register_cross, register_middleware
from .init import init_app
EOF

cat>apps/v1_api/__init__.py<<EOF
from fastapi import FastAPI

from apps.libs.exception import register_exception
from .endpoints import user


def register_routers(app: FastAPI):
    """register routers"""

    app.include_router(user.router, prefix="/users", tags=["User"])


def init_sub_app(app: FastAPI):
    """mount sub app"""

    api_app: FastAPI = FastAPI()

    register_exception(api_app)
    register_routers(api_app)
    app.mount(path="/api/v1", app=api_app, name="v1_api")

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

    logger.error("start error, this is".center(60, "*"))
    logger.error(f"{method} {url}")
    logger.error(message)
    logger.error("end error".center(60, "*"))


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
        content = {"code": StatusCode.BadRequest, "message": exc.detail, "data": None}
        return JSONResponse(content=content, status_code=exc.status_code, headers=exc.headers)

    @app.exception_handler(AssertionError)
    async def assert_exception_handle(request: Request, exc: AssertionError):
        """catch Python AssertError"""

        exc_str = " ".join(exc.args)
        log_message(request.method, request.url, exc_str)
        content = {"code": StatusCode.AssertValidatorError, "message": exc_str, "data": None}
        return JSONResponse(content=content, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)

    @app.exception_handler(ValidationError)
    async def db_validation_exception_handle(request: Request, exc: ValidationError):
        """catch tortoise-orm ValidatorError"""

        exc_str = "|".join(exc.args)
        log_message(request.method, request.url, exc.args)
        content = {"code": StatusCode.ValidatorError, "message": exc_str, "data": None}
        return JSONResponse(content=content, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        """catch FastAPI RequestValidationError"""

        exc_str = f"{exc}".replace("\n", " ").replace("   ", " ")
        log_message(request.method, request.url, exc)
        # content = exc.errors()
        content = {"code": StatusCode.RequestValidatorError, "message": exc_str, "data": None}
        return JSONResponse(content=content, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)

    @app.exception_handler(Exception)
    async def exception_handle(request: Request, exc: Exception):
        """catch other exception"""

        log_message(request.method, request.url, traceback.format_exc())
        content = {"code": StatusCode.ServerError, "message": str(exc), "data": None}
        return JSONResponse(content=content, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)
EOF

cat>apps/libs/middleware.py<<EOF
import time

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.gzip import GZipMiddleware

from conf.const import StatusCode

MiddlewareCodeContents = {
    405: {"code": StatusCode.MethodNotAllowed, "message": "METHOD_NOT_ALLOWED", "data": None},
    406: {"code": StatusCode.NotAcceptable, "message": "NOT_ACCEPTABLE", "data": None},
    408: {"code": StatusCode.RequestTimeout, "message": "REQUEST_TIMEOUT", "data": None},
    411: {"code": StatusCode.LengthRequired, "message": "LENGTH_REQUIRED", "data": None},
    413: {"code": StatusCode.EntityTooLarge, "message": "REQUEST_ENTITY_TOO_LARGE", "data": None},
    414: {"code": StatusCode.RequestUriTooLong, "message": "REQUEST_URI_TOO_LONG", "data": None},
    431: {
        "code": StatusCode.HeaderFieldsTooLarge,
        "message": "REQUEST_HEADER_FIELDS_TOO_LARGE",
        "data": None
    }
}


def register_cross(app: FastAPI):
    """deal Cross Origin Resource Sharing"""

    app.add_middleware(
        CORSMiddleware,
        allow_methods=["*"],
        allow_headers=["*"],
        allow_credentials=True,
        allow_origin_regex="https?://.*",
        expose_headers=["X-TOKEN", "X-Process-Time"]
    )

    app.add_middleware(
        GZipMiddleware,
        minimum_size=500,
        compresslevel=9
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

        response.headers["X-Process-Time"] = str((time.time() - start_time) * 1000)
        return response
EOF

cat>apps/libs/init.py<<EOF
from fastapi import FastAPI

from .database import init_db
from .middleware import register_cross, register_middleware
from apps.v1_api import init_sub_app as init_v1_api_app


def init_app(app: FastAPI) -> FastAPI:

    init_db(app)
    register_cross(app)
    register_middleware(app)

    init_v1_api_app(app)

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


class AbstractModel(Model):
    id = UnsignedBigIntField(pk=True)
    uid = fields.CharField(max_length=100)
    created_at = fields.DatetimeField(auto_now_add=True, null=False)
    updated_at = fields.DatetimeField(auto_now=True, null=False)
    is_deleted = fields.BooleanField(null=False, default=False)

    @property
    def created_time(self) -> str:
        created_at = getattr(self, "created_at")
        return created_at.strftime("%Y-%m-%d %H:%M:%S") if created_at else ""

    @property
    def updated_time(self) -> str:
        updated_at = getattr(self, "updated_at")
        return updated_at.strftime("%Y-%m-%d %H:%M:%S") if updated_at else ""

    class Meta:
        abstract = True


class ModelMixin(object):
    __slots__ = ()

    def __init__(self, **kwargs):
        pass

    @staticmethod
    def to_dict(instance, selects: tuple = None, excludes: tuple = None) -> dict:

        if not hasattr(instance, "_meta"):
            raise AssertionError("<%r> does not have attribute for _meta" % instance)

        if selects:
            return {i: getattr(instance, i) for i in selects}
        elif excludes:
            return {i: getattr(instance, i) for i in instance._meta.fields if i not in excludes}
        else:
            return {i: getattr(instance, i) for i in instance._meta.fields}

    async def async_to_dict(self, selects: tuple = None, excludes: tuple = None, second_attrs: dict = None) -> dict:
        """response dict data of instance serialize

        selects: ("id", "name")
        excludes: ("created_at", "updated_at")
        second_attrs: {"owner": ["id", "name"], "agency": ["id", "name"]}
        """

        results = self.to_dict(self, selects=selects, excludes=excludes)
        if second_attrs:
            for attr, columns in second_attrs.items():
                results.update({attr: self.to_dict(await getattr(self, attr), selects=columns)})

        return results

    def sync_to_dict(self, selects: tuple = None, excludes: tuple = None, second_attrs: dict = None) -> dict:
        """response dict data of instance serialize

        selects: ("id", "name")
        excludes: ("created_at", "updated_at")
        second_attrs: {"owner": ["id", "name"], "agency": ["id", "name"]}
        """

        results = self.to_dict(self, selects=selects, excludes=excludes)
        if second_attrs:
            for attr, columns in second_attrs.items():
                results.update({attr: self.to_dict(getattr(self, attr), selects=columns)})

        return results
EOF

cat>apps/modules/resource.py<<EOF
from tortoise.models import Model

from extensions import NotFound


class ResourceOp():
    def __init__(self, model, pk):
        self.model: Model = model
        self.pk = pk

    async def instance(self, is_deleted=None):
        _instances = self.model.filter(id=self.pk)
        if is_deleted is not None:
            _instance = _instances.filter(is_deleted=is_deleted)
        _instance = await _instances.first()
        if not _instance:
            raise NotFound(message=f"Model = {self.model.__name__}, pk = {self.pk} is not exists")
        return _instances, _instance
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
        return f"{uuid.uuid4().hex}"

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

cat>conf/__init__.py<<EOF
from .const import PaginateConst
from .const import StatusCode
EOF

cat>conf/const.py<<EOF
class StatusCode(object):
    Success = 10000

    BadRequest = 40000
    Unauthorized = 40100
    Forbidden = 40300
    NotFound = 40400
    MethodNotAllowed = 40500
    NotAcceptable = 40600
    RequestTimeout = 40800
    LengthRequired = 41100
    EntityTooLarge = 41300
    RequestUriTooLong = 41400
    ValidatorError = 42200
    RequestValidatorError = 42201
    AssertValidatorError = 42202
    Locked = 42300
    HeaderFieldsTooLarge = 43100

    ServerError = 45000
    UnknownError = 45001


class PaginateConst:
    DefaultNum = 1
    DefaultSize = 10

    MinNum = 1
    MaxSize = 40


class EnvConst:

    TEST = "test"
    PRD = "prd"
    DEV = "dev"
EOF

cat>conf/settings.py<<EOF
from functools import lru_cache
from typing import Optional

from pydantic import BaseModel


class LogSetting(BaseModel):
    LEVEL: Optional[str] = "DEBUG"
    PATH: str
    STDOUT: Optional[bool] = True
    ROTATION: Optional[str] = "00:00"
    RETENTION: Optional[str] = "30 days"
    COMPRESSION: Optional[str] = None


class ServiceSetting(BaseModel):
    # openapi swagger
    INCLUDE_IN_SCHEMA: Optional[bool] = True


class AuthenticSetting(BaseModel):
    ADMIN_SECRETS: str
    ADMIN_TOKEN_EXP_DELTA: Optional[int] = 864000


class RedisSetting(BaseModel):
    HOST: Optional[str] = "127.0.0.1"
    PORT: Optional[int] = 6379
    PASSWD: Optional[str] = None
    SOCKET_TIMEOUT: Optional[float] = 10
    SOCKET_CONNECT_TIMEOUT: Optional[float] = 10
    MAX_CONNECTIONS: Optional[int] = None
    USER: Optional[str] = None


class DBSetting(BaseModel):
    POOL_RECYCLE: Optional[str] = 1000

    USER: Optional[str] = "root"
    PASSWD: str
    HOST: Optional[str] = "127.0.0.1"
    PORT: Optional[int] = 3306
    DATABASE: str
    MAX_SIZE: Optional[int] = 5


class Setting(BaseModel):
    log: LogSetting
    service: ServiceSetting
    authentic: AuthenticSetting
    redis: RedisSetting
    db: DBSetting


class ORMSetting:
    def __init__(self, db: DBSetting):
        self.db = db

    def _base_orm_conf(self, apps: dict) -> dict:
        return {
            "connections": {
                "default": {
                    "engine": "tortoise.backends.mysql",
                    "credentials": {
                        "host": self.db.HOST,
                        "port": self.db.PORT,
                        "user": self.db.USER,
                        "password": self.db.PASSWD,
                        "database": self.db.DATABASE,
                        "minsize": 1,
                        "maxsize": self.db.MAX_SIZE,
                        "charset": "utf8mb4",
                        "pool_recycle": self.db.POOL_RECYCLE
                    }
                }
            },
            "apps": apps,
            "use_tz": False,
            "timezone": "Asia/Shanghai"
        }

    @property
    @lru_cache
    def orm_link_conf(self) -> dict:
        orm_apps_settings = {
            "models": {
                "models": [
                    "aerich.models",
                    "apps.models.models"
                ],
                "default_connection": "default",
            }
        }
        return self._base_orm_conf(orm_apps_settings)

    @property
    def orm_migrate_conf(self) -> dict:
        orm_apps_settings = {
            "models": {
                "models": [
                    "aerich.models",
                    "apps.models.models"
                ],
                "default_connection": "default",
            }
        }
        return self._base_orm_conf(orm_apps_settings)

    @property
    def orm_test_migrate_conf(self) -> dict:
        orm_apps_settings = {
            "models": {
                "models": [
                    "aerich.models",
                    "apps.models.models"
                ],
                "default_connection": "default",
            }
        }
        return self._base_orm_conf(orm_apps_settings)
EOF

cat>conf/product.toml<<EOF
[log]
LEVEL = "INFO"
PATH = "/home/logs/x.log"

[db]
POOL_RECYCLE = 1800

USER = "root"
PASSWD = "passwd"
HOST = "127.0.0.1"
PORT = 3306
DATABASE = "database"
MAX_SIZE = 500

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

cat>conf/test.toml<<EOF
[log]
LEVEL = "INFO"
PATH = "/home/logs/x-test.log"

[db]
POOL_RECYCLE = 1800

USER = "root"
PASSWD = "passwd"
HOST = "127.0.0.1"
PORT = 3306
DATABASE = "database"
MAX_SIZE = 500

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
__all__ = ["logger"]

import sys
from pathlib import Path

from loguru import logger

from config import LogConfig

level = LogConfig.LEVEL.upper()
Path(LogConfig.PATH).parent.mkdir(exist_ok=True)

logger.remove()
logger.add(
    LogConfig.PATH, level=level, rotation=LogConfig.ROTATION, retention=LogConfig.RETENTION, backtrace=True,
    diagnose=True, enqueue=True, compression=LogConfig.COMPRESSION
)
if LogConfig.STDOUT:
    logger.add(sys.stdout, level=level, backtrace=True, diagnose=True, enqueue=True)
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

            logger.info("start request".center(60, "*"))
            logger.info(f"{request.method} {request.url}")

            methods = ["POST", "PUT", "PATCH"]
            content_type = request.headers.get("content-type", "")

            if request.method in methods and "application/json" in content_type:
                try:
                    params = await request.json()
                    if params:
                        logger.info(params)
                except JSONDecodeError:
                    logger.error("encounter JSONDecodeError")
                except UnicodeDecodeError:
                    logger.error("encounter UnicodeDecodeError")
            logger.info("end request".center(60, "*"))
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
    code: int = StatusCode.BadRequest
    message: str = ""
    data: NoneType = "null"


class UnauthorizedSchema(BaseModel):
    code: int = StatusCode.Unauthorized
    message: str = ""
    data: NoneType = "null"


class ForbiddenSchema(BaseModel):
    code: int = StatusCode.Forbidden
    message: str = ""
    data: NoneType = "null"


class NotFoundSchema(BaseModel):
    code: int = StatusCode.NotFound
    message: str = ""
    data: NoneType = "null"


class ValidatorErrorSchema(BaseModel):
    code: int = StatusCode.ValidatorError
    message: str = ""
    data: NoneType = "null"


ErrorSchema = {
    400: {
        "model": BadRequestSchema,
        "description": "BadRequest"
    },
    401: {
        "model": UnauthorizedSchema,
        "description": "Unauthorized"
    },
    403: {
        "model": ForbiddenSchema,
        "description": "Forbidden"
    },
    404: {
        "model": NotFoundSchema,
        "description": "NotFound"
    },
    422: {
        "model": ValidatorErrorSchema,
        "description": "Request Parameters Validator"
    }
}


class SchemaMixin(BaseModel):
    code: int = StatusCode.Success
    message: str = "success"
    data: Optional[Any]


class NormalSchema(SchemaMixin):
    """default normal common return schema"""

    data: Optional[str] = "success"


class FilterParserMixin(BaseModel):
    """search list data"""

    page: Optional[int] = Query(PaginateConst.DefaultNum, title="page", gte=PaginateConst.MinNum)
    page_size: Optional[int] = Query(PaginateConst.DefaultSize, title="page_size", gte=1, lte=PaginateConst.MaxSize)
EOF

cat>extensions/exceptions.py<<EOF
from typing import Optional, Any, Dict

from fastapi import status
from starlette.exceptions import HTTPException

from conf import StatusCode
from .schema import SchemaMixin


class BaseHTTPException(HTTPException):
    MESSAGE = None
    STATUS_CODE = status.HTTP_400_BAD_REQUEST
    CODE = StatusCode.BadRequest

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
    CODE = StatusCode.BadRequest


class Unauthorized(BaseHTTPException):
    STATUS_CODE = status.HTTP_401_UNAUTHORIZED
    CODE = StatusCode.Unauthorized


class Forbidden(BaseHTTPException):
    STATUS_CODE = status.HTTP_403_FORBIDDEN
    CODE = StatusCode.Forbidden


class NotFound(BaseHTTPException):
    STATUS_CODE = status.HTTP_404_NOT_FOUND
    CODE = StatusCode.NotFound


class MethodNotAllowed(BaseHTTPException):
    STATUS_CODE = status.HTTP_405_METHOD_NOT_ALLOWED
    CODE = StatusCode.MethodNotAllowed


class Locked(BaseHTTPException):
    STATUS_CODE = status.HTTP_423_LOCKED
    CODE = StatusCode.Locked
EOF

cat>extensions/paginate.py<<EOF
from tortoise.queryset import QuerySet, MODEL

from conf.const import PaginateConst


class Pagination(object):
    def __init__(
            self,
            query: QuerySet,
            page: int = PaginateConst.DefaultNum,
            page_size: int = PaginateConst.DefaultSize
    ) -> None:
        self.query = query
        self.page = page
        self.page_size = page_size

    def items(self) -> QuerySet[MODEL]:
        return self.query.offset((self.page - 1) * self.page_size).limit(self.page_size)
EOF

cat>extensions/response.py<<EOF
from typing import Any

from extensions import logger
from conf.const import StatusCode


def resp_success(code: int = StatusCode.success, message: str = "", print_msg: str = "", data: Any = None):
    if print_msg:
        pass
    else:
        if message and message != "success":
            print_msg = message

    if print_msg:
        logger.info(print_msg)

    return {"code": code, "message": message, "data": data}
EOF


# redis_ext
mkdir redis_ext
touch redis_ext/__init__.py
touch redis_ext/base.py
touch redis_ext/lock.py
touch redis_ext/sms.py

cat>redis_ext/base.py<<EOF
import os
import threading
from typing import Union, Optional

from redis.typing import EncodableT, ExpiryT
from redis.asyncio import Redis
from redis.asyncio.connection import ConnectionPool

from config import RedisConfig
from conf.const import EnvConst

REDIS_CONNECTION_PARAMS = {
    "host": RedisConfig.HOST,
    "port": RedisConfig.PORT,
    "password": RedisConfig.PASSWD,
    "socket_timeout": RedisConfig.SOCKET_TIMEOUT,
    "socket_connect_timeout": RedisConfig.SOCKET_CONNECT_TIMEOUT,
    "socket_keepalive": True,
    "encoding": "utf-8",
    "decode_responses": True,
    "max_connections": RedisConfig.MAX_CONNECTIONS,
    "username": RedisConfig.USER
}


class Pool:
    cache = dict()
    lock = threading.Lock()
    instance = None

    def __init__(self, db: int = 0) -> None:
        self.db = db

    def __new__(cls, db: int = 0):
        db = str(db)

        with cls.lock:
            if not cls.instance:
                cls.instance = super().__new__(cls)

            if not cls.cache.get(db):
                cls.cache[db] = ConnectionPool(db=db, **REDIS_CONNECTION_PARAMS)

            return cls.instance

    def pool(self):
        return self.cache.get(str(self.db))


class BaseRedis(object):
    DB = 0
    PREFIX_KEY = ""

    def __init__(self) -> None:
        self._name = None

        # TODO FIX
        if os.environ.get("CODE_ENV") == EnvConst.TEST:
            self.client: Redis = Redis(connection_pool=ConnectionPool(db=self.DB, **REDIS_CONNECTION_PARAMS))
        else:
            self.client: Redis = Redis(connection_pool=Pool(self.DB).pool())

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = f"{self.PREFIX_KEY}:{value}"

    def expire(self, seconds: ExpiryT, nx: bool = False, xx: bool = False, gt: bool = False, lt: bool = False):
        """
        Set an expired flag on key ''name'' for ''time'' seconds. ''time''
        can be represented by an integer or a Python timedelta object.

            NX -> Set expiry only when the key has no expiry
            XX -> Set expiry only when the key has an existing expiry
            GT -> Set expiry only when the new expiry is greater than current one
            LT -> Set expiry only when the new expiry is less than current one
        """

        return self.client.expire(name=self.name, time=seconds, nx=nx, xx=xx, gt=gt, lt=lt)

    def delete(self):
        """Delete one or more keys specified by ''names''"""

        return self.client.delete(self.name)

    def exists(self):
        """Returns the number of ''names'' that exist"""

        return self.client.exists(self.name)

    def get(self):
        """
        Return the value at key ''name'', or None if the key doesn"t exist
        """

        return self.client.get(name=self.name)

    def _incr_by(self, amount: int = 1):
        """value += amount"""

        return self.client.incrby(name=self.name, amount=amount)

    def set(
            self,
            value: EncodableT,
            ex: Union[ExpiryT, None] = None,
            px: Union[ExpiryT, None] = None,
            nx: bool = False,
            xx: bool = False,
            get: bool = False
    ):
        """Set the value at key ''name'' to ''value''

        ''ex'' sets an expired flag on key ''name'' for ''ex'' seconds.

        ''px'' sets an expired flag on key ''name'' for ''px'' milliseconds.

        ''nx'' if set to True, set the value at key ''name'' to ''value'' only
            if it does not exist.

        ''xx'' if set to True, set the value at key ''name'' to ''value'' only
            if it already exists.

        ''get'' if True, set the value at key ''name'' to ''value'' and return
            the old value stored at key, or None if the key did not exist.
            (Available since Redis 6.2)
        """

        return self.client.set(name=self.name, value=value, ex=ex, px=px, nx=nx, xx=xx, get=get)

    def _hash_set(self, key: Optional[str] = None, value: Optional[str] = None, mapping: Optional[dict] = None):
        """
        Set ''key'' to ''value'' within hash ''name'',
        ''mapping'' accepts a dict of key/value pairs that that will be
        added to hash ''name''.
        Returns the number of fields that were added.
        """

        return self.client.hset(name=self.name, key=key, value=value, mapping=mapping)

    async def _hash_get_values(self, keys: list = None):
        """hash, Returns a list of values ordered identically to ''keys''"""

        if not keys:
            return await self.client.hgetall(name=self.name)

        response = await self.client.hmget(name=self.name, keys=keys)

        result = dict()
        for index, key in enumerate(keys):
            result[key] = response[index]

        return result

    def _hash_del_key(self, keys: list):
        """hash, Delete ''keys'' from hash ''name''

        have * need list         ([key, key, key])
            hdel(*[key, key, key])
        no   * need multiple key (key, key, key)
            hdel(key, key, key)
        """

        self.client.hdel(self.name, *keys)

    def _lis_push(self, values: list, is_right: bool = True):
        """list, Push ''values'' into the head or tail of the list ''name'', depend on is_right flag"""

        if is_right:
            return self.client.rpush(self.name, *values)
        else:
            return self.client.lpush(self.name, *values)

    def _list_left_range(self, start: int = 0, end: int = -1):
        """list, Return a slice of the list ''name'' between position ''start'' and ''end''"""

        return self.client.lrange(name=self.name, start=start, end=end)

    def _list_set(self, index: int, value: str):
        """list, Set element at ''index'' of list ''name'' to ''value''"""

        return self.client.lset(name=self.name, index=index, value=value)
EOF

cat>redis_ext/lock.py<<EOF
import time
from typing import Union, Tuple

from .base import BaseRedis


class BaseLockRedis(BaseRedis):
    """full key: Lock:{key}"""

    DB = 2
    PREFIX_KEY = "Lock"
    TIMEOUT = 3600

    async def get_lock(self) -> Tuple[bool, Union[str, None]]:
        """get the lock"""

        current_time = time.time()
        current_value = current_time + self.TIMEOUT

        lock = await self.set(value=current_value, ex=self.TIMEOUT, nx=True)
        if lock:
            # origin is not exists now set it OK
            return True, str(current_value)

        # it had exists, get_old_value
        old_lock = await self.get()

        if old_lock and current_time > float(old_lock):
            # had expired
            # set new and get old
            old_value = await self.set(value=current_value, ex=self.TIMEOUT, get=True)
            if not old_value or old_lock == old_value:
                return True, str(current_value)
            return False, None
        return False, None


class OrderLockRedis(BaseLockRedis):
    """full key: orderLock:{key}"""

    DB = 2
    PREFIX_KEY = "orderLock"
    TIMEOUT = 3600
EOF


# scripts
mkdir scripts
mkdir scripts/init_data
touch scripts/__init__.py


# services
mkdir services
touch services/__init__.py
mkdir services/mqtt_services
mkdir services/celery_services
touch services/celery_services/__init__.py
touch services/celery_services/application.py
touch services/celery_services/config.py
touch services/mqtt_services/__init__.py
touch services/mqtt_services/client.py

cat>services/celery_services/config.py<<EOF
from kombu import Queue, Exchange


class MQConfig:
    USER = ""
    PASSWD = ""
    HOST = ""
    PORT = ""


class RedisConfig:
    USER = ""
    PASSWD = ""
    HOST = ""
    PORT = ""


DefaultExchangeType = "direct"


class QueueNameConst:
    default = "celery-default-queue"
    test = "celery-test-queue"
    pay = "celery-pay-queue"


class ExchangeConst:
    default = "celery-default-exchange"
    test = Exchange(name="celery-test-exchange", type=DefaultExchangeType)
    pay = Exchange(name="celery-pay-exchange", type=DefaultExchangeType)


class RoutingKeyConst:
    default = "celery-default-routing"
    test = "celery-test-routing"
    pay = "celery-pay-routing"


class CeleryConfig:
    # 1，任务队列 代理设置
    broker_url = f"amqp://{MQConfig.USER}:{MQConfig.PASSWD}@{MQConfig.HOST}:{MQConfig.PORT}"

    # 2，结果存储 默认，无
    result_backend = f"redis://{RedisConfig.USER}:{RedisConfig.PASSWD}@{RedisConfig.HOST}:{RedisConfig.PORT}/0"

    # 3，存储结果，过期时间为 一小时
    result_expires = 60 * 60

    # 4，禁用 UTC
    enable_utc = False

    # 5，时区
    timezone = "Asia/Shanghai"

    # 6，允许的接收的内容类型/序列化程序的白名单 默认，json
    accept_content = ["json"]
    # 允许结果后端的内容类型/序列化程序的白名单 默认，与 accept_content 相同
    # result_accept_content

    # 7，以秒为单位的任务硬时间限制 默认，无
    # task_time_limit = 100

    # 8，default
    # 消息没有路由或没有指定自定义队列使用的默认队列名称，默认值，celery
    task_default_queue = QueueNameConst.default
    # 当没有为设置中键指定自定义交换时使用的交换的名称
    task_default_exchange = ExchangeConst.default
    # 当没有为设置中键指定自定义交换类型时使用的交换类型，默认值，direct
    task_default_exchange_type = DefaultExchangeType
    # 当没有为设置中键指定自定义路由键时使用的路由键
    task_default_routing_key = RoutingKeyConst.default

    # 9，消息路由 使用 kombu.Queue
    task_queues = (
        Queue(name=QueueNameConst.pay, exchange=ExchangeConst.pay, routing_key=RoutingKeyConst.pay),
    )

    # 10，路由列表把任务路由到队列的路由
    task_routes = {
        "pay": {"exchange": ExchangeConst.pay.name, "routing_key": RoutingKeyConst.pay}
    }
EOF

cat>services/mqtt_services/client.py<<EOF
"""
pip install paho-mqtt
"""

import json
import ssl
from typing import Any, List

from paho.mqtt.client import Client, MQTTv311, MQTTv5
from paho.mqtt.subscribeoptions import SubscribeOptions
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.reasoncodes import ReasonCodes

from extensions import logger


class MqttClient:
    def __init__(
            self,
            host: str,
            port: int,
            client_id: str,
            username: str,
            password: str,
            keep_alive: int = 600,
            version: int = MQTTv311,
            properties: Properties = None,
            ca_cert: str = None,
            cert_path: str = None,
            key_path: str = None,
            keyfile_passwd: str = None
    ) -> None:
        """__init__

        Args:
            host (str): MQTT Server Host.
            port (int): MQTT Server Port.
            client_id (str): Client ClientID.
            username (str): Client UserName.
            password (str): Client Password.
            keep_alive (int, optional): MQTT Connect KeepAlive.
            version (int, optional): MQTT Version. Defaults to MQTTv311.
            properties (Properties, optional): when MQTTv5 use this. Defaults to None.
            ca_cert (str, optional): TLS CA cert file. Defaults to None.
            cert_path (str, optional): TLS Client cert path. Defaults to None.
            key_path (str, optional): TLS Client key path. Defaults to None.
            keyfile_passwd (str, optional): TLS Client key password. Defaults to None.
        """

        self._host = host
        self._port = port
        self.client_id = client_id
        self.username = username
        self.password = password
        self._keep_alive = keep_alive
        self.version = version
        self.properties = properties
        self._ca_cert = ca_cert
        self._cert_path = cert_path
        self._key_path = key_path
        self._keyfile_passwd = keyfile_passwd

        self.unique = self.client_id or self.username

        self.client = Client(client_id=self.client_id, protocol=version)
        if version == MQTTv5:
            self.client.on_connect = self.on_connect_v5
            self.client.on_disconnect = self.on_disconnect_v5
            self.client.on_subscribe = self.on_subscribe_v5
            self.client.on_unsubscribe = self.on_unsubscribe_v5
        else:
            self.client.on_connect = self.on_connect_v3
            self.client.on_disconnect = self.on_disconnect_v3
            self.client.on_subscribe = self.on_subscribe_v3
            self.client.on_unsubscribe = self.on_unsubscribe_v3

        self.client.on_publish = self.on_publish
        self.client.on_message = self.on_message

    def set_ssl_context(self):
        """set SSL context to use SSL"""

        if self._ca_cert and self._cert_path and self._key_path:
            context = ssl.SSLContext(protocol=ssl.PROTOCOL_TLS)
            context.check_hostname = False
            context.load_cert_chain(certfile=self._cert_path, keyfile=self._key_path, password=self._keyfile_passwd)
            context.load_verify_locations(self._ca_cert)
            context.verify_mode = ssl.CERT_REQUIRED

            self.client.tls_set_context(context=context)

    @staticmethod
    def set_properties(version: int, params: dict):

        if version in [MQTTv5]:
            properties = Properties(packetType=PacketTypes.CONNECT)
            for key, value in params.items():
                setattr(properties, key, value)
            return properties
        return None

    def connect(self):
        """Verify and Connect to a remote broker."""

        self.client.username_pw_set(self.username, self.password)
        self.set_ssl_context()

        return self.client.connect(self._host, self._port, self._keep_alive, properties=self.properties)

    def disconnect(self, rc: ReasonCodes = None):
        """Disconnect a connected client from the broker.
        rc: (MQTT v5.0 only) a ReasonCodes instance setting the MQTT v5.0
        rc to be sent with the disconnect.  It is optional, the receiver
        then assuming that 0 (success) is the value.
        """

        return self.client.disconnect(rc, properties=self.properties)

    def loop_start(self):
        """loop start"""

        self.client.loop_start()

    def loop_stop(self, force: bool = False):
        """loop stop"""

        self.client.loop_stop(force=force)

    def loop_forever(self, timeout: float = 1.0):
        """loop forever"""

        self.client.loop_forever(timeout=timeout)

    def add_callback(self, topic, callback):
        """Register a message callback for a specific topic."""

        self.client.message_callback_add(topic, callback)

    def subscribe(self, topic: str, qos: int = 0, options: SubscribeOptions = None):
        """Subscribe the client to one or more topics.

        options
            if MQTTv311 then None
            if MQTTv5 then example options=SubscribeOptions(qos=2)
        """

        return self.client.subscribe(topic, qos=qos, options=options, properties=self.properties)

    def unsubscribe(self, topic: str):
        """Unsubscribe the client to one or more topics."""

        return self.client.unsubscribe(topic=topic, properties=self.properties)

    def publish(self, topic, payload=None, qos: int = 0, retain: bool = False, flag: bool = True):
        """Publish a message on a topic."""

        logger.info(f"publish topic {topic}".center(60, "*"))
        if flag:
            payload = json.dumps(payload, ensure_ascii=False)

        return self.client.publish(
            topic=topic, payload=payload, qos=qos, retain=retain, properties=self.properties
        )

    def on_connect_v3(self, client: Client, userdata: Any, flags: dict, rc: int):
        """Define the connected callback implementation."""

        logger.info(f"{self.unique} on_connect flags {flags} rc {rc}".center(60, "*"))

    def on_connect_v5(
            self, client: Client, userdata: Any, flags: dict, reason_code: ReasonCodes, properties: Properties
    ):
        """Define the connected callback implementation."""

        logger.info(f"{self.unique} on_connect flags {flags} rc {reason_code}".center(60, "*"))

    def on_disconnect_v3(self, client: Client, userdata: Any, rc: int):
        """If implemented, called when the client disconnects from the broker."""

        logger.info(f"{self.unique} on_disconnected rc {rc}".center(60, "*"))

    def on_disconnect_v5(self, client: Client, userdata: Any, rc: int, properties: Properties = None):
        """If implemented, called when the client disconnects from the broker."""

        logger.info(f"{self.unique} on_disconnected rc {rc}".center(60, "*"))

    def on_subscribe_v3(self, client: Client, userdata: Any, mid: int, granted_qos: tuple):
        """Define the subscribed callback implementation."""

        logger.info(f"{self.unique} on_subscribe: mid {mid} granted_qos {granted_qos}".center(60, "*"))

    def on_subscribe_v5(
            self, client: Client, userdata: Any, mid: int, reason_codes: List[ReasonCodes], properties: Properties
    ):
        """Define the subscribed callback implementation."""

        _reason_codes = [rc.json() for rc in reason_codes]
        logger.info(f"{self.unique} on_subscribe: mid {mid} reason_codes {_reason_codes}".center(60, "*"))

    def on_unsubscribe_v3(self, client: Client, userdata: Any, mid: int):
        """Define the un subscribe callback implementation."""

        logger.info(f"{self.unique} on_unsubscribe: mid {mid}".center(60, "*"))

    def on_unsubscribe_v5(
            self, client: Client, userdata: Any, mid: int, properties: Properties, reason_code: ReasonCodes
    ):
        """Define the un subscribe callback implementation."""

        logger.info(f"{self.unique} on_unsubscribe: mid {mid} reason_code {reason_code}".center(60, "*"))

    def on_publish(self, client: Client, userdata: Any, mid: int):
        """Define the published message callback implementation."""

        logger.info(f"{self.unique} on_publish: mid {mid}".center(60, "*"))

    def on_message(self, client: Client, userdata: Any, message):
        """Define the message received callback implementation. use this when can't match message_callback_add"""

        logger.info(f"{self.unique} on_message topic {message.topic}".center(60, "*"))
        payload = message.payload.decode("utf-8")
        logger.info(payload)
EOF

# tests
mkdir tests
mkdir tests/fixture_data
touch tests/__init__.py
touch tests/conftest.py
touch tests/pre_write_data.py
touch tests/token.py
touch tests/utils.py

cat>tests/__init__.py<<EOF
import sys
from pathlib import Path

BASE_DIR = Path(__file__).parent.parent

sys.path.append(BASE_DIR)

from config import ORM_TEST_MIGRATE_CONF
from apps.application import create_app
from extensions import NotFound, BadRequest
EOF

cat>tests/conftest.py<<EOF
__all__ = [
    "client"
]

from typing import Generator

import pytest
from tortoise import run_async
from fastapi.testclient import TestClient

from tests import create_app
from tests.pre_write_data import create_database, delete_database
from tests.token import generate_token, remove_token


@pytest.fixture(scope="session", autouse=True)
def client() -> Generator:
    try:
        # create db and create table and create data
        run_async(create_database())

        # set token into environ
        run_async(generate_token())
        with TestClient(create_app()) as test_client:
            yield test_client
    finally:
        # drop db
        run_async(delete_database())

        remove_token()
EOF

cat>tests/pre_write_data.py<<EOF
__all__ = [
    "create_database", "delete_database"
]

from pathlib import Path

from tortoise import Tortoise

from tests import ORM_TEST_MIGRATE_CONF, BASE_DIR
from tests import User, AdminUser, Book, Car, Order, Phone, Question
from tests.utils import JsonFileOperator


def _build_instances(model_class, file: str):
    file = Path(BASE_DIR).joinpath(f"tests/fixture_data/{file}")
    for per_dic in JsonFileOperator(file).read():
        yield model_class(**per_dic)


async def _write_data():
    batch_size = 5

    await User.bulk_create(objects=_build_instances(User, "users.json"), batch_size=batch_size)
    await AdminUser.bulk_create(objects=_build_instances(AdminUser, "admin_users.json"), batch_size=batch_size)
    await Book.bulk_create(objects=_build_instances(Book, "books.json"), batch_size=batch_size)
    await Car.bulk_create(objects=_build_instances(Car, "cars.json"), batch_size=batch_size)
    await Order.bulk_create(objects=_build_instances(Order, "orders.json"), batch_size=batch_size)
    await Phone.bulk_create(objects=_build_instances(Phone, "phones.json"), batch_size=batch_size)
    await Question.bulk_create(objects=_build_instances(Question, "questions.json"), batch_size=batch_size)
    print("write pre data over")


async def create_database():
    """create database and create tables"""

    # create database
    await Tortoise.init(config=ORM_TEST_MIGRATE_CONF, _create_db=True)
    print("create database over")

    # create tables
    await Tortoise.generate_schemas()
    print("create tables over")

    await _write_data()


async def delete_database():
    """drop database"""

    # link to database
    await Tortoise.init(config=ORM_TEST_MIGRATE_CONF)

    # drop database
    await Tortoise._drop_databases()
    print("drop database over")
EOF

cat>tests/token.py<<EOF
__all__ = [
    "generate_token"
]

import os

from tortoise import Tortoise

from tests import AuthenticConfig, ORM_TEST_MIGRATE_CONF
from tests import NotFound, BadRequest, TokenResolver
from tests import User, AdminUser
from tests import TokenRedis

EXTEND_MODEL_MAP = {"AdminUser": AdminUser, "User": User}


async def authentic_test(cellphone: str, extend_model: str = "AdminUser"):
    model_class = EXTEND_MODEL_MAP.get(extend_model)
    if not model_class:
        raise BadRequest(message=f"Model {extend_model} error")

    user = await User.get_or_none(cellphone=cellphone, is_delete=False)
    if not user:
        raise NotFound(f"User User.cellphone = {cellphone} is not exists or is deleted")

    extend_user = await model_class.filter(user_id=user.id, is_delete=False).first()
    if not extend_user:
        raise NotFound(message=f"{extend_model} User.cellphone = {cellphone} is not exists or is deleted")

    token, login_time, token_expired = TokenResolver.encode_auth_token(user.id, cellphone, extend_user.id, extend_model)
    extend_user.login_time = login_time
    extend_user.token_expired = token_expired
    await extend_user.save()

    token_redis_op = TokenRedis()
    token_redis_op.name = f"{user.cellphone}:{extend_model}:{extend_user.id}"
    await token_redis_op.set(token, ex=AuthenticConfig.ADMIN_TOKEN_EXP_DELTA)

    return token


async def generate_token():
    """create a super_admin_user token and save it into os.environ"""

    # link to database
    await Tortoise.init(config=ORM_TEST_MIGRATE_CONF)

    os.environ["AdminUserTestToken"] = await authentic_test("10000000001")


def remove_token():
    """remove os env test token

    remove from redis
    """

    os.environ["AdminUserTestToken"] = ""
    print("remove env AdminUserTestToken")
EOF


# tools
mkdir tools
touch tools/__init__.py
touch tools/cli.py
touch tools/worker.py


cat>tools/cli.py<<EOF
import asyncio
import os
import sys
import json
from pathlib import Path
from typing import Union, List
from functools import wraps
from datetime import datetime

import click
from tortoise import Tortoise
from tortoise.transactions import atomic
from loguru import logger

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from config import ORM_LINK_CONF
from apps.models import User, Book, AdminUser, Car, Order, Phone, Question

model_map = {
    'User': User,
    'Book': Book,
    'AdminUser': AdminUser,
    'Car': Car,
    'Order': Order,
    'Phone': Phone,
    'Question': Question
}


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


@click.command(help='create init data')
@click.pass_context
@click.option('-f', '--file', type=str, required=True, help='json file')
@click.option('-m', '--model', type=str, required=True, help='Database Model')
@coro
@atomic()
async def init_data(ctx: click.Context, file: str, model: str):
    """create users"""

    model_class = model_map.get(model)
    if not model_class:
        raise Exception(f'no this Model {model}')

    params = _read_json_file(file)

    await create(model_class, params)
    logger.info('create users over')


@click.command(help='create migration template file')
@click.pass_context
@click.option('-d', '--migration_dir', type=str, required=True, help='Migration Dir')
@click.option('-n', '--migration_name', type=str, required=True, help='Migration Name')
def create_migration_file(ctx: click.Context, migration_dir: str, migration_name: str):
    """create a blank migration sql file"""

    migration_dir_p = Path(migration_dir)
    if not migration_dir_p.is_dir():
        migration_dir_p.mkdir(exist_ok=True)

    now = datetime.now().strftime('%Y%m%d%H%M%S')

    # get the new latest sql file index
    exists_sql_file_names = os.listdir(migration_dir_p)
    if exists_sql_file_names:
        latest_sql_file_index = max([int(f.split(bytes('_'))[0]) for f in exists_sql_file_names])
        new_latest_sql_file_index = latest_sql_file_index + 1
    else:
        new_latest_sql_file_index = 0

    # get the new latest sql file name
    new_latest_sql_file_name = f'{new_latest_sql_file_index}_{now}_{migration_name}.sql'

    data = """-- upgrade --

    -- downgrade --

    """

    with open(migration_dir_p.joinpath(new_latest_sql_file_name), 'w', encoding='utf-8') as f:
        f.write(data)

    logger.info(f'create {migration_name} done')


@click.command(help='find some target data')
@click.pass_context
@click.option('-d', '--dirs', nargs='*', type=str, required=True, help='dirs')
@click.option('-t', '--targets', nargs='+', required=True, help='target')
def find(ctx: click.Context, dirs: list, targets: list):
    """
    Traverse all files under single or multiple folders (UTF-8 files)
    Find the target in the file according to the targets in the parameter
    If found, record the target and file path in the log
    """

    directions = list()

    if dirs:
        for d in dirs:
            _d = Path(d).absolute()
            if _d.is_dir():
                directions.append(_d)
    else:
        logger.error(f'your input -d/--dirs {dirs} error')
        return

    if not directions:
        logger.error(f'your input -d/--dirs {directions} error')
        return

    if not targets:
        logger.error(f'your input -t/--targets {targets} error')
        return

    def read_file(path: Union[str, Path]) -> str:
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return f.read()
        except:
            return ''

    for direction in directions:
        for root, _, files in os.walk(direction):
            for file in files:
                file_path = Path(root, file)
                txt = read_file(file_path)
                for target in targets:
                    if target in txt:
                        logger.info(f'{file_path}')

    logger.info('find done')


cli.add_command(init_data)
cli.add_command(create_migration_file)

if __name__ == '__main__':
    cli()
EOF


cat>tools/worker.py<<EOF
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
# bullseye
deb http://mirrors.aliyun.com/debian/ bullseye main non-free contrib
deb http://mirrors.aliyun.com/debian-security/ bullseye-security main
deb http://mirrors.aliyun.com/debian/ bullseye-updates main non-free contrib
deb http://mirrors.aliyun.com/debian/ bullseye-backports main non-free contrib
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
# ==========================================================================
server_id = 1
bind_address = 0.0.0.0
port = 3306

authentication_policy = mysql_native_password

default_time_zone=+08:00

character_set_server = utf8mb4
# character
init_connect = "SET NAMES utf8mb4"
# order
collation_server = utf8mb4_0900_ai_ci


# ==========================================================================
# wait_timeout = 3600
# interactive_timeout = 3600
connect_timeout = 10
net_read_timeout = 30
net_write_timeout = 60


# ==========================================================================
# log
# ==========================================================================
# error log 开启后不再向 stdout 打印
log_error = /var/log/mysql/error.log

# binlog
log_bin = /var/log/mysql/master-bin.log
# binlog index
# log_bin_index = /var/log/mysql/master-bin.log.index
# binlog 模式, DEFAULT=row
binlog_format = mixed
# 事务能够使用的最大 binlog 缓存空间 DEFAULT=32K MAX= MIN=4K
binlog_cache_size = 1M
# binlog 文件最大空间，达到该大小时切分文件 DEFAULT=1073741824 1G, MAX=1G, MIN=4K
max_binlog_size = 256M
# BINLOG 保存时间，秒数
binlog_expire_logs_seconds = 864000

# 启用慢查询日志
slow_query_log = ON
# 慢查询检测时间
long_query_time = 20
# 慢查询文件
slow_query_log_file = /var/log/mysql/slow.log
# 记录 更改表、分析表、检查表、创建索引、删除索引、优化表和修复表 慢查询
log_slow_admin_statements = ON


# ==========================================================================
# relay log
# ==========================================================================
relay_log = /var/log/mysql/relay.log
relay_log = /var/log/mysql/relay.log.index
# 从库从主库复制的数据写入从库 binlog 日志 DEFAULT=OFF
log_slave_updates = ON
# 最大 relay log size DEFAULT=0 MAX=1073741824 1G MIN=0
max_relay_log_size = 256M
# 自动清空不再需要的中继日志 DEFAULT=ON
relay_log_purge = ON
# 重启 slave 时删除所有 relay log, 通过 SQL 重放的位置点去重新拉取日志 DEFAULT=OFF
relay_log_recovery = ON


# ==========================================================================
# replica
# ==========================================================================
# 副本集类型
replica_parallel_type = LOGICAL_CLOCK
# 副本worker num
replica_parallel_workers = 4
# slave 上 commit 顺序保持一致
replica_preserve_commit_order = ON


# ==========================================================================
# 连接
# ==========================================================================
# 允许的最大并发客户端数, MAX=100000, MIN=1, DEFAULT=151
max_connections = 2000
# 操作系统可用于 mysqld 的文件描述符数量, MAX=平台, MIN=0, DEFAULT=5000
open_files_limit = 5000
# 所有线程的打开表数, MAX=524288, MIN=1, max(open_files_limit - 10 - max_connections) / 2, 400), DEFAULT=4000
table_open_cache = 4000


# ==========================================================================
# 缓冲区, 连接查询, 索引, 读取缓冲, 排序缓冲, 临时表, 用户表
# ==========================================================================
# DEFAULT 256K
join_buffer_size = 2M
# DEFAULT 8M
key_buffer_size = 64M
# DEFAULT 128K
read_buffer_size = 1M
# DEFAULT 256K
sort_buffer_size = 2M
# DEFAULT 16M
tmp_table_size = 256M
# DEFAULT 16M
max_heap_table_size = 256M


# ==========================================================================
read_only = OFF


# ==========================================================================
# 数据包
# ==========================================================================

# 数据包最大大小, 单位: 字节, MAX=1073741824 1G, MIN=1024
max_allowed_packet = 64M


# ==========================================================================
# InnoDB
# ==========================================================================
# 缓冲区 default=128M
innodb_buffer_pool_size = 256M
# 异步 I/O 子系统
# innodb_use_native_aio = NO
# 读线程数
innodb_read_io_threads = 16
# 写线程数
innodb_write_io_threads = 16
# 并行查询
innodb_parallel_read_threads = 16
EOF


# other
mkdir tmp logs

touch .gitignore .dockerignore
touch README.md CHANGELOG.md Dockerfile Makefile docker-entrypoint.sh
touch config.py server.py pyproject.toml

cat>Makefile<<EOF

up_require:
	pip install -r mirrors/requirements.txt

up_dev_require:
	pip install -r mirrors/requirements-dev.txt

test:
	CODE_ENV=test pytest --rootdir ./tests -s

run:
	uvicorn main:app --reload
EOF

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

## 1

### 1.0

#### 1.0.1

## 0

### 0.1

#### 0.0.1
EOF

cat>config.py<<EOF
__all__ = [
    "BASE_DIR",
    "Config",
    "ORM_LINK_CONF",
    "ORM_MIGRATE_CONF",
    "ORM_TEST_MIGRATE_CONF",
    "LogConfig",
    "ServiceConfig",
    "AuthenticConfig",
    "RedisConfig"
]

import os
from pathlib import Path
from functools import lru_cache

import pytomlpp

from conf.const import EnvConst
from conf.settings import (
    Setting, ORMSetting
)

BASE_DIR = Path(__file__).absolute().parent


@lru_cache()
def get_settings() -> Setting:
    code_env = os.environ.get("CODE_ENV", EnvConst.PRD)

    if code_env == EnvConst.TEST:
        p = Path(BASE_DIR).joinpath("conf/test.local.toml")
        if not p.is_file():
            p = Path(BASE_DIR).joinpath("conf/test.toml")
    else:
        p = Path(BASE_DIR).joinpath("conf/product.local.toml")
        if not p.is_file():
            p = Path(BASE_DIR).joinpath("conf/product.toml")

    if not p.is_file():
        raise Exception("config no exists")

    settings = Setting.parse_obj(pytomlpp.load(p))
    return settings


Config = get_settings()

ORM_LINK_CONF = ORMSetting(Config.db).orm_link_conf
ORM_MIGRATE_CONF = ORMSetting(Config.db).orm_migrate_conf
ORM_TEST_MIGRATE_CONF = ORMSetting(Config.db).orm_test_migrate_conf

LogConfig = Config.log
ServiceConfig = Config.service
AuthenticConfig = Config.authentic
RedisConfig = Config.redis
EOF

cat>server.py<<EOF
from apps.application import create_app

app = create_app()
EOF

cat>pyproject.toml<<EOF
[tool.aerich]
tortoise_orm = "config.ORM_MIGRATE_CONF"
location = "./migrations"
src_folder = "./."

[tool.pytest.ini_options]
filterwarnings = ["ignore::DeprecationWarning"]
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


cat>README.md<<EOF
# README

>

## keywords

## environment

- [require packets for work](./mirrors/requirements.txt)
- [require packets for dev and test](./mirrors/requirements-dev.txt)

## command

## dir and file

### project file

- [A Global Config File](./config.py)
- [Dir or File for Using Cython cythonize](./build.txt)
- [The Program Entry File](./server.py)
- [Dockerfile](./Dockerfile)
- [Makefile](./Makefile)
- [The Conf File of Aerich](./pyproject.toml)
- [The Change Log of Different Version for This Project](./CHANGELOG.md)

### some tools

- [The Script of Insert Some Data Into Database](./scripts/insert.py)
- [The Extend uvicorn Worker](./tools/worker.py)

## deploy and dir

### build and run

### the project dir example

.
├── api
├── conf
│   ├── xxx_api
│   │   ├── product.local.toml
│   │   ├── test.local.toml
│   │   ├── docker-entrypoint.sh
│   │   └── gunicorn_config.py
│   ├── mysql
│   │   └── my.cnf
│   └── redis
│       └── redis.conf
├── data
│   ├── mysql
│   │   └── data
│   └── redis
│       └── data
│           └── dump.rdb
├── docker-compose.yml
└── logs
    └── xxx_api
        ├── x.log
        ├── x-test.log
        ├── x-local.log
        └── x-local-test.log
EOF
