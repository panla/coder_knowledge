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
| Text(65536) | 对应mysql medinumtext | 2^24 - 1 bytes |
| Text(16777216) | 对应mysql longtext | 2^32 - 1 bytes |

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

    
class Account(db.Model):
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


created_at = db.Column(db.DateTime, server_default=func.now(), comment='创建时间')
updated_at = db.Column(db.DateTime, server_default=text('CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP'), comment='更新时间')
is_delete = db.Column(db.Boolean, server_default=text('0'), nullable=False, comment='是否标记删除')

```

- server_default=func.now() 设置自动获取时间
- server_default=text('CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP') 设置自动更新(不用这个ORM模型也能自动更新)

## ORM

### 查

```text
from sqlalchemy import func
from sqlalchemy import distinct


Car.query.filter_by(id=1)
Car.query.filter(Car.id == 1)
Car.query.all()

# 对 BaseQuery 排序
Car.query.order_by(Car.id.desc())

# 对 BaseQuery 分页
Cars.query.paginate(page, pagesize).items

# 对 BaseQuery 分组查询
Car.query.with_entities(Car.price, func.count(Car.price)).group_by(Car.price).all()
    返回 [(1, 214), (2, 422)]

# 去重统计数量，根据报告表，找到该 agency_id 的用户数量，因为一个用户可能在多个 agency 作报告，所以需要去重。
Report.query.filter_by(agency_id=1).with_entities(func.count(distinct(Report.owner_id))).first()
    返回 (count,)

# 连接查询
Report.query.join(User, Report.owner_id == User.id).filter(User.gender == 1).all()
    外连接 outerjoin
    使用 join 后就只能使用 filter() 特别指定用哪个模型的哪个字段来搜索

```

执行顺序

```text
GROUP BY子句必须出现在WHERE子句之后，ORDER BY子句之前。
HAVING语句必须在ORDER BY子句之后。

where -> group by -> order by -> having
```

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
