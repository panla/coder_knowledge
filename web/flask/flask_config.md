# Flask 配置

## 读取配置

### 读取 Python 文件

```python
from flask import Flask

app = Flask(__name__)
app.config.from_pyfile('')

```

### 读取 Python 类

```python
from flask import Flask


class Config(object):
    pass


app = Flask(__name__)
app.config.from_object(Config)

```

## 配置内容

### Flask 相关

```python

DEBUG = True

# 密钥用于会话 cookie 的安全签名，并可用于应用或者扩展的其他安全需求
# python -c 'import os; import base64; print(base64.b64encode(os.urandom(16)))'
SECRET_KEY = b'veG8UyFAdwQFGKr/ByHXnw=='

# url_for 可以为应用生成一个单独的外部 URL 而不是一个请求情景
SERVER_NAME = 'https://example.com'

# 返回的 JSON 数据，主要用于处理中文变成 ascii 的问题
JSON_AS_ASCII = False

# flask_restful 的同上配置
RESTFUL_JSON = dict(ensure_ascii=False)

```

### 数据库相关

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
