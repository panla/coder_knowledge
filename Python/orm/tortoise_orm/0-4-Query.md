# Query

[toc]

## 参考

- [Query](https://tortoise-orm.readthedocs.io/en/latest/query.html)

## 对象所包含方法

### Model

```text
update_from_dict()                                                  使用提供的字典更新当前模型
    await car.update_from_dict(car_item.dict())
    await car.save()

Model.select_for_update()       -> QuerySet().select_for_update()
Model.register_listener(signal, listen)                             为特殊信号注册监听器到当前模型类

async refresh_from_db()                                             从 db 刷新最新数据
aysnc Model.fetch_for_list()                                        为提供的模型对象列表获取相关模型
async fetch_related()                                               获取相关字段, ????
    await User.fetch_related("email", "username")

async save()                                                        创建更新当前对象
async delete()
async Model.create()                                                创建对象并返回
async Model.get_or_create()                                         如果存在则获取，不存则创建，返回实例
Model.update_or_create()                                            一种使用给定 kwargs 更新对象的便捷方法
async Model.bulk_create()                                           批量创建

Model.first()                   -> QuerySet().first()               生成返回第一条记录的 QuerySet
Model.filter()                  -> QuerySet().filter()              生成一个应用了过滤器的 QuerySet
Model.all()                     -> QuerySet().all()                 返回完整 QuerySet
Model.get()                     -> QuerySet().get()                 使用提供的过滤器参数获取模型类型的单个记录
Model.get_or_none()             -> QuerySet().get_or_none()         使用提供的过滤器参数或 None 获取模型类型的单个记录

Model.exclude()                 -> QuerySet().exclude()
Model.annotate()                -> QuerySet().annotate()            使用额外的函数/聚合/表达式求结果集
Model.exists()                  -> QuerySet().exists()
```

### QuerySet

```text
filter()                        -> "QuerySet[MODEL]"
exclude()                       -> "QuerySet[MODEL]"
all()                           -> "QuerySet[MODEL]"
first()                         -> QuerySetSingle[Optional[MODEL]]
get()                           -> QuerySetSingle[MODEL]
get_or_none()                   -> QuerySetSingle[Optional[MODEL]]

values_list()                   -> "ValuesListQuery"                使QuerySet返回元组列表
values()                        -> ValuesQuery                      使QuerySet返回字典
distinct()                      -> "QuerySet[MODEL]"                与 values 结合

only()                          -> QuerySet[MODEL]
select_related()                -> QuerySet[MODEL]

order_by()                      -> "QuerySet[MODEL]"
limit()                         -> "QuerySet[MODEL]"
offset()                        -> "QuerySet[MODEL]"
annotate()                      -> "QuerySet[MODEL]"
group_by()                      -> "QuerySet[MODEL]"

select_for_update()             -> "QuerySet[MODEL]"

force_index()                   -> QuerySet[MODEL]
use_index()                     -> QuerySet[MODEL]
using_db()                      -> QuerySet[MODEL]

prefetch_related()              -> QuerySet[MODEL]

async explain()                 -> Any

delete()                        -> DeleteQuery
update()                        -> UpdateQuery
count()                         -> CountQuery
exists()                        -> ExistsQuery

sql()
```

### all

```python
async for car in Car.all():
    print(car.id)

for q in await Car.all():
    print(q.id)
```

## select

### 双下划线

```text
not
in, not_in
gte, gt, lte, lt
range
isnull, not_isnull
contains, icontains
startswith, istartswith endswith, iendswith
iexact
search

await Project.filter(name__in=['a', 'b'])
await Project.filter(user__name__in=['a', 'b'])

{FKNAME}_id__isnull
```

### `prefetch_related`

```python
from tortoise.query_utils import Prefetch

# 复杂预取
question = await Question.all().prefetch_related(
    Prefetch('owner', queryset=User.filter(id=1), to_attr='owner').first())

# question 就包含了 owner 对象
# 需要有 ForeignKeyField 关系 db_constraint=True/False

await Question.all().prefecth_related('owner')
```

### filter date

```python
class DatePart(Enum):
    year = "YEAR"
    quarter = "QUARTER"
    month = "MONTH"
    week = "WEEK"
    day = "DAY"
    hour = "HOUR"
    minute = "MINUTE"
    second = "SECOND"
    microsecond = "MICROSECOND"

await Team.filter(created_at__year=2020)
await Team.filter(created_at__month=11)
await Team.filter(created_at__day=5)
```

## annotate Aggregates functions

### annotate functionms

函数对 Field 的每个实例应用转换。

```python
from tortoise.functions import Trim, Lower, Upper, Coalesce， Length

# 去除两端空白
await Tournament.annotate(clean_name=Trim('name'))).filter(clean_name='tournament')
await Tournament.annotate(name_upper=Upper('name'))).filter(name_upper='TOURNAMENT')
await Tournament.annotate(name_lower=Lower('name'))).filter(name_lower='tournament')
# 接收至少两个字段名 返回第一个 不为空的值
await Tournament.annotate(desc_clean=Coalesce('desc', ''))).filter(desc_clean='')
```

### aggregates functions

聚合应用于整个列，并且经常与分组一起使用

```python
from tortoise.functions import Count, Sum, Max, Min, Avg

# This query will fetch all tournaments with 10 or more events, and will
# populate filed `.events_count` on instances with corresponding value
await Tournament.annotate(events_count=Count('events')).filter(events_count__gte=10)
```

### 自定义函数

## Q F

### Q

```python
from tortoise.query_utils import Q

await Event.filter(Q(name='aaa') | Q(name='b'))
# | & ~
# join_type='OR' 'AND' 'NOT'
```

### F

它可以引用模型字段值并使用它们执行数据库操作，而实际上不必将它们从数据库中提取到 Python 内存中

```python
from tortoise.expressions import F

await User.filter(id=1).update(balance=F('balance') - 10)
await User.filter(id=1).update(balance=F('balance') + F('award'))

```
