# Query

[Query](https://tortoise-orm.readthedocs.io/en/latest/query.html)

## 对象所包含方法

### Model

```text
update_from_dict()

async save()
async delete()
async fetch_related()
async refresh_from_db()
async Model.get_or_create()
Model.select_for_update() -> QuerySet()select_for_update
Model.update_or_create()
async Model.create()
async Model.bulk_create()

Model.first() -> QuerySet()first()
Model.filter() -> QuerySet()filter()
Model.exclude() -> QuerySet().exclude()
Model.all() -> QuerySet().all()
Model.get() -> QuerySet().get()

Model.exists() -> QuerySet().exists()
Model.annotate() -> QuerySet().annotate()
Model.get_or_none() -> QuerySet().get_or_none()
aysnc Model.fetch_for_list()
```

### QuerySet

```text
filter() -> "QuerySet[MODEL]"
exclude() -> "QuerySet[MODEL]"
all() -> "QuerySet[MODEL]"

only() -> QuerySet[MODEL]
select_related() -> QuerySet[MODEL]

first() -> QuerySetSingle[Optional[MODEL]]
get() -> QuerySetSingle[MODEL]
get_or_none() -> QuerySetSingle[Optional[MODEL]]

order_by() -> "QuerySet[MODEL]"

limit() -> "QuerySet[MODEL]"
offset() -> "QuerySet[MODEL]"

distinct() -> "QuerySet[MODEL]"
select_for_update() -> "QuerySet[MODEL]"
annotate() -> "QuerySet[MODEL]"
group_by() -> "QuerySet[MODEL]"

force_index() -> QuerySet[MODEL]
use_index() -> QuerySet[MODEL]
prefetch_related() -> QuerySet[MODEL]

using_db() -> QuerySet[MODEL]

async explain() -> Any    

values_list() -> "ValuesListQuery"
values() -> ValuesQuery

delete() -> DeleteQuery
update() -> UpdateQuery
count() -> CountQuery
exists() -> ExistsQuery
```

## select

双下划线

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
```

### `prefetch_related`

```python
from tortoise.query_utils import Prefetch

question = await Question.all().prefetch_related(
    Prefetch('owner', queryset=User.filter(id=1), to_attr='owner').first())

# question 就包含了 owner 对象
# 需要有 ForeignKeyField 关系 db_constraint=True/False
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

## annotate functions

```python
from tortoise.functions import Count, Trim, Lower, Upper, Coalesce

# This query will fetch all tournaments with 10 or more events, and will
# populate filed `.events_count` on instances with corresponding value
# 去除两端空白
await Tournament.annotate(clean_name=Trim('name'))).filter(clean_name='tournament')
await Tournament.annotate(name_upper=Upper('name'))).filter(name_upper='TOURNAMENT')
await Tournament.annotate(name_lower=Lower('name'))).filter(name_lower='tournament')
# 接收至少两个字段名 返回第一个 不为空的值
await Tournament.annotate(desc_clean=Coalesce('desc', ''))).filter(desc_clean='')

Length

```

## annotate Aggregates functions

```python
from tortoise.functions import Count, Sum, Max, Min, Avg

await Tournament.annotate(events_count=Count('events')).filter(events_count__gte=10)

```

## Q

```python
from tortoise.query_utils import Q

await Event.filter(Q(name='aaa') | Q(name='b'))
# | & ~
# join_type='OR' 'AND' 'NOT'
```

## F

```python
from tortoise.expressions import F

await User.filter(id=1).update(balance=F('balance') - 10)
await User.filter(id=1).update(balance=F('balance') + F('award'))

```
