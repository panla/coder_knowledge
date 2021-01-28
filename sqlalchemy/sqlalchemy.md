# SQLAlchemy

[Flask_SQLAlchemy](https://flask-sqlalchemy.palletsprojects.com/en/2.x/)
[Flask_SQLAlchemy 中文](http://www.pythondoc.com/flask-sqlalchemy/quickstart.html)
[SQLAlchemy](https://www.osgeo.cn/sqlalchemy/)

## 字段

### 字段类型

| 类型 | 说明 | 其他 |
| :-: | :-: | :-: |
| DECIMAL(10, 4) | 定点型 | 总共10，小数部分最多4位 |
| Boolean | 布尔类型 |  |
| DateTime | 日期时间格式 |  |
| Integer/INTEGER | 整数类型 | 不能 Integer(11) 一般为32位 |
| SmallInteger/SMALLINT | 对应mysql smallint | 一般为16位 |
| String | 对应mysql varchar |  |
| Text | 对应mysql tinytext | 2^16 - 1 bytes |

### 字段属性

字段属性

```text
primary_key = True  主键约束
unique = True     唯一约束
index = True     索引
nullable = False   null约束
default         默认值，创建更新表时不会有，在新创建数据时会有
```

### 联合约束

```python
from sqlalchemy import UniqueConstraint


# 对字段 pharmacy_id 和 name 联合唯一约束
__table_args__ = (
    UniqueConstraint('pharmacy_id', 'name'),
)

```

```python
from sqlalchemy.orm import validates


class Medicine():
    pass


# 对 pharmacy_id 和 name 作校验，使创建和更新时 pharmacy_id name 联合唯一
@validates('name')
def validate_name(self, key, name):
    if Medicine.query.filter(
            Medicine.id != self.id, Medicine.pharmacy_id == self.pharmacy_id, Medicine.name == name).first():
        raise Exception('this pharmacy_id name id already exist')
    return name

```

## 连接

### 一般配置

```python
# 应用于连接的数据库 URI
SQLALCHEMY_DATABASE_URI = "mysql:pymysql://user:passwd@host:port/db?charset=utf8mb4"

# 绑定多个数据库
SQLALCHEMY_BINDS = {
    'key': 'uri'
}

SQLALCHEMY_TRACK_MODIFICATIONS = False

SQLALCHEMY_ENGINE_OPTIONS = {
    'pool_recycle': 50,
    'pool_size': 20,
    'max_overflow': -1
}

```

### `create_engine` 参数

```text
pool_size   连接池大小，默认 5
max_overflow  超过 pool_size 后还能创建的连接
pool_recycle  多长时间重建连接
```

### 映射连接

```python
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy import create_engine

DB_URL = "mysql:pymysql://user:passwd@host:port/db?charset=utf8mb4"
SQLALCHEMY_ENGINE_OPTIONS = {
    'pool_recycle': 50,
    'pool_size': 20,
    'max_overflow': -1
}

db = SQLAlchemy(session_options={"bind": create_engine(DB_URL, **SQLALCHEMY_ENGINE_OPTIONS)})


def db_session_commit():
    # 保存数据异常捕获，回滚

    try:
        db.session.commit()
    except Exception:
        db.session.rollback()
        raise


class ModelMixin(object):

    __slots__ = ()

    def __init__(self):
        pass

    def save(self):
        """保存数据"""

        db.session.add(self)
        db_session_commit()
        return self

    def delete(self, commit=True):
        """删除数据"""

        db.session.delete(self)
        if commit:
            db_session_commit()

    def update(self, **kwargs):
        """修改数据"""

        commit = False
        for k, v in kwargs.items():
            if hasattr(self, k) and getattr(self, k) != v:
                commit = True
                setattr(self, k, v)
        if commit:
            db_session_commit()
        return commit

    def serialize(self, excludes=None, selects=None):
        """
        返回json格式数据，序列化
        :param excludes: 不想返回的字段，接收一个列表，ex.: ['password', 'name']
        :param selects: 想要返回的字段，接收一个列表，ex.: ['id', 'age']
        :return:
        """

        if not hasattr(self, '__table__'):
            raise AssertionError('<%r> does not have attribute for __table__' % self)
        elif selects:
            return {i: getattr(self, i) for i in selects}
        elif excludes:
            return {i.name: getattr(self, i.name) for i in self.__table__.columns if i.name not in excludes}
        else:
            return {i.name: getattr(self, i.name) for i in self.__table__.columns}

    @classmethod
    def get(cls, **kwargs):
        return cls.query.filter_by(**kwargs).first()

    @classmethod
    def list(cls, **kwargs):
        return cls.query.filter_by(**kwargs)


class Account(db.Model, ModelMixin):
    """医生账户表"""

    __bind_key__ = "key"
    __tablename__ = "accounts"

```

## 数据迁移

### Flask-Migrate 设置

`migrations/env.py`

```text
def run_migrations_online():
    ...
    with connectable.connect() as connection:
    context.configure(
        connection=connection,
        target_metadata=target_metadata,
        process_revision_directives=process_revision_directives,
        compare_type=True,# 检查字段类型
        compare_server_default=True,# 如果从 default -> server_default 需要使用一次
        **current_app.extensions['migrate'].configure_args
    )

    with context.begin_transaction():
        context.run_migrations()
```

- `compare_type=True` 检查字段类型
- `compare_server_default` 检查 server_default 设置

### 时间类

```python
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy import func
from sqlalchemy import text
from sqlalchemy import create_engine

DB_URL = "mysql:pymysql://user:passwd@host:port/db?charset=utf8mb4"
SQLALCHEMY_ENGINE_OPTIONS = {
    'pool_recycle': 50,
    'pool_size': 20,
    'max_overflow': -1
}

db = SQLAlchemy(session_options={"bind": create_engine(DB_URL, **SQLALCHEMY_ENGINE_OPTIONS)})

id = db.Column(db.BigInteger, primary_key=True)
created_at = db.Column(db.DateTime, nullable=False, server_default=text('CURRENT_TIMESTAMP'), comment='创建时间')
updated_at = db.Column(
    db.DateTime, nullable=False, server_default=text('CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP'),
    comment='更新时间'
    )
is_delete = db.Column(db.Boolean, nullable=False, server_default=text('0'), comment='是否标记删除')
name = db.Column(db.String(30), index=True, nullable=False, comment='名称')
nickname = db.Column(db.String(30), nullable=False, server_default='', comment='昵称')

```

- server_default=text('CURRENT_TIMESTAMP') 设置自动获取时间
- server_default=text('CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP') 设置自动更新(不用这个ORM模型也能自动更新)
- server_default='' 默认值为 ''

## 唯一，索引

```text
index=True
    KEY `ix_tbas_name` (`name`),
unique=True
    UNIQUE KEY `name` (`name`),
index=True, unique=True
    UNIQUE KEY `ix_tbcs_name` (`name`),

唯一约束由唯一索引来实现，所以也是索引的一种
```
