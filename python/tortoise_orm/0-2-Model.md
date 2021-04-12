# models

## usages

### 基类

```python
class AbstractModel(Model):

    id = fields.BigIntField(pk=True)
    created_at = fields.DatetimeField(null=False, auto_now_add=True, description='创建时间')
    updated_at = fields.DatetimeField(null=False, auto_now=True, description='更新时间')
    is_delete = fields.BooleanField(null=False, default=False, description='删除标识')

    class Meta:
        abstract = True

```

### Meta

```python
class Car(AbstractModel):

    brand = fields.CharField(max_length=100, null=False, index=True, description='品牌')

    class Meta:
        app = 'default'
        table = 'cars'
        table_description = '汽车表'
        unique_together = (('column_a', 'column_b'), )
        indexes = ('column_a', 'column_b')
        # indexes = (('column_a', 'column_b'), ('column_c', 'column_d'))
        ordering = ['name', '-id']
        # 分组查询，annotate 默认排序设置不生效

```

### Model 提供的方法

```text
    all()
    annotate()
    exclude()       排除某些结果集
    first()
    filter()
    annotate()
    get()

    create()
    exists()
    bulk_create()

async function
    delete()
    save()
```
