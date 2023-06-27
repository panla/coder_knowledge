# docker-compose 示例

[TOC]

## 示例

com 目录下

### app.py

```python
import time

import redis
from flask import Flask

app = Flask(__name__)
cache = redis.Redis(host='172.18.0.2', port=6379)
# host 可以使用容器名称 redis

def get_hit_count():
    retries = 5
    while True:
        try:
            return cache.incr('hits')
        except redis.exceptions.ConnectionError as exc:
            if retries == 0:
                raise exc
            retries -= 1
            time.sleep(0.5)

@app.route('/')
def hello():
    count = get_hit_count()
    return 'Hello World! I have been seen {} times.\n'.format(count)
```

### web Dockerfile

```text
FROM python:3.7-alpine
WORKDIR /code
COPY requirements.txt requirements.txt

ENV FLASK_APP=app.py
ENV FLASK_RUN_HOST=0.0.0.0

RUN apk add --no-cache gcc musl-dev linux-headers && pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

EXPOSE 5000
COPY . .
CMD ["flask", "run"]
```

### yaml 示例

```yaml
version: "3.9"
services:
  web:
    build: .
    ports:
      - "5000:5000"
    networks:
      mynet_test:
        ipv4_address: "172.18.0.3"
    depends_on:
      - redis
  redis:
    image: "redis:alpine"
    networks:
      mynet_test:
        ipv4_address: "172.18.0.2"

networks:
  mynet_test:
    external: true
```

### 运行

```bash
docker-compose up
```

访问 `127.0.0.1:5000`

### 状态

```text
# 网络，手动创建
docker network create --driver bridge --gateway=172.18.0.1 --subnet=172.18.0.0/16 mynet_test

# 镜像
com_web
redis:alpine
python:3.7-alpine

# 容器
image           name
redis:alpine    com_redis_1
com_web         com_web_1
```

未配置网络时，也可以 redis.Redis(host='redis')
