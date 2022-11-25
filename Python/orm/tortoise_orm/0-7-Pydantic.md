# Tortoise-orm 结合 Pydantic

[toc]

## 参考

- [文档](https://tortoise-orm.readthedocs.io/en/latest/examples/pydantic.html)

```python
from tortoise.contrib.pydantic import PydanticModel, PydanticListModel
from tortoise.contrib.pydantic import pydantic_model_creator, pydantic_queryset_creator
```

```python
p = await EventPydantic.from_totoise_orm(await Event.get(name='test'))
p.json()
```
