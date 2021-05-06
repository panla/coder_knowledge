# Query

[Query](https://tortoise-orm.readthedocs.io/en/latest/query.html)

## QuerySet

### QuerySet 所拥有的方法

`tortoise.quertset.QuerySet`

```text
filter() -> QuerySet
all() -> QuerySet

exclude() -> QuerySet
only() -> QuerySet
order_by() -> QuerySet
group_by() -> QuerySet
limit() -> QuerySet
offset() -> QuerySet
distinct() -> QuerySet

select_for_update() -> QuerySet
annotate() -> QuerySet

prefetch_related() -> QuerySet
    instance.fetch_related
select_related() -> QuerySet

using() -> QuerySet

first() -> QuerySetSingle
get() -> QuerySetSingle
get_or_none() -> QuerySetSingle

exists() -> ExistsQuery
values() -> ValuesQuery
values_list() -> ValuesListQuery
count() -> CountQuery
delete() -> DeleteQuery
update() -> UpdateQuery

explain()

```

### AwaitableQuery

```text
resolve_filters()
resolve_ordering()
sql()
```

### QuerySetSingle

```text
prefetch_related()
annotate() -> QuerySetSingle
only() -> QuerySetSingle
values() -> ValuesQuery
values_list() -> ValuesListQuery
```

### Model 实例的方法

`tortoise.models.Model`

```text
clone() -> Model
update_from_dict() -> Model
save()
delete()
fetch_related()
```

### `prefetch_related`

```python
question = await Question.all().prefetch_related(
    Prefetch('owner', queryset=User.filter(id=1), to_attr='owner').first())

# question 就包含了 owner 对象
# 需要有 ForeignKeyField 关系 db_constraint=True/False
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
