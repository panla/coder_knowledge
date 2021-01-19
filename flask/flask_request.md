# Flask 请求

## request

### request 属性

```python
from flask import request

# 请求的路径
path = request.path

# 请求的完整路径
full_path = request.full_path

# 请求的host，127.0.0.1:5000
host = request.host

# 请求的host_url，http://127.0.0.1:5000/
host_url = request.host_url

# 请求的请求头
headers = request.headers

# 请求的方法
method = request.method

# 获取上传的多个文件
files = request.files.getlist('files')

# 获取上传的单个文件
file = request.file.get('file')

# 获取查询参数
request.args.get('name')

```

### 获取真实 IP

```text
nginx 中
    proxy_set_header Host $host:$server_port;
    proxy_set_header X-Real-IP $remote_addr;
    proxy_set_header REMOTE-HOST $remote_addr;
    proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;

flask 中
    request.headers['X-Real-Ip']
```

### 文件对象

```text
file.filename
```

## 全局变量 g

## 请求钩子

应用级 和 蓝图级 blueprint

### 第一个请求前

before_first_request

### 请求前

在每次请求前执行，可以用来做 TOKEN 校验

before_request

### 请求后

每次请求后执行（如果未抛错）

after_request

### 每次请求后

每次请求后执行，接收一个参数: 错误信息，如果有相关错误抛出，需要 DEBUG=False 才会收到

teardown_request

### 全局异常处理

`register_error_handler(Exception, process_exception)`

## 跨域

```python
from flask import request
from flask import make_response


def cross_domain_access_before():
    """在请求前，设置路由跨域请求"""

    if request.method == 'OPTIONS':
        response = make_response()
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Headers'] = '*'
        response.headers['Access-Control-Max-Age'] = 24 * 60 * 60
        response.headers['Access-Control-Allow-Methods'] = "GET, POST, DELETE, PATCH"
        return response


def cross_domain_access_after(response):
    """在请求后，设置返回headers头"""

    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Headers'] = '*'
    return response

```
