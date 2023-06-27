# Aerich 迁移

[TOC]

## 参考

- [Aerich](https://github.com/tortoise/aerich)
- [文档参考](https://tortoise-orm.readthedocs.io/en/latest/migration.html)

## config

```text
modules={
    "models": [
        'apps.models.model',
        'aerich.models'
        ]
    }
```

## command

```python
TORTOISE_ORM = {
    "connections": {
        "default": "mysql://root:123456@127.0.0.1:3306/test"
        },
    "apps": {
        "models": {
            "models": ["tests.models", "aerich.models"],
            "default_connection": "default",
        },
    },
}
```

```bash
# 初始化，指定配置文件
aerich init -t config.TORTOISE_ORM --location ./migrations

# init db
aerich init-db

# 自动生成sql语句
aerich migrate --name init
aerich migrate --name alter_books_add_column_sn

# 执行到最新的迁移
aerich upgrade
```
