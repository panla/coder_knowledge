# Router 读写

[toc]

## 参考

- [文档](https://tortoise-orm.readthedocs.io/en/latest/router.html)

## 定义

```python
from typing import Type

from tortoise.models import Model


class Router:
    def db_for_read(self, model: Type[Model]):
        return 'salve'

    def db_for_write(self, model: Type[Model]):
        return 'master'
```

## config

```python
TORTOISE_CONFIG = {
    'connections': {
        'master': 'sqlite:///tmp/test.db',
        'slave': 'sqlite:///tmp/test.db'
        },
    'apps': {
        'models': {
            'models': ['__main__'],
            'default_connection': 'master',
        }
    },
    'routers': ['apps.mixins.router.Router'],
    'use_tz': False,
    'timezone': 'Asia/Shanghai',
}
```
