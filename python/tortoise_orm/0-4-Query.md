# Query

[Query](https://tortoise-orm.readthedocs.io/en/latest/query.html)

## QuerySet

```text
all
annotate
count
delete
distinct
exclude
exists
explain
filter
first
get
get_or_nonegroup_by
limit
offset
only
order_by

prefetch_related
    instance.fetch_related
resolve_filters
resolve_ordering
update
using_db
values
values_list
```

## select

```text
not
in, not_in
gte, gt, lte, lt
range
isnull, not_isnull
contains, icontains
startswith, istartswith
endswith, iendswith
iexact
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
