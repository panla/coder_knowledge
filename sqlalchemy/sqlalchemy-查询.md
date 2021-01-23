# SQLALchemy 查询

## 一

```text
from sqlalchemy import func
from sqlalchemy import distinct


Car.query.filter_by(id=1)
Car.query.filter(Car.id == 1)
Car.query.all()

# 对 BaseQuery 排序
Car.query.order_by(Car.id.desc())

# 对 BaseQuery 分页
Cars.query.paginate(page, pagesize).items

# 对 BaseQuery 分组查询
Car.query.with_entities(Car.price, func.count(Car.price)).group_by(Car.price).all()
    返回 [(1, 214), (2, 422)]

# 去重统计数量，根据报告表，找到该 agency_id 的用户数量，因为一个用户可能在多个 agency 作报告，所以需要去重。
Report.query.filter_by(agency_id=1).with_entities(func.count(distinct(Report.owner_id))).first()
    返回 (count,)

# 连接查询
Report.query.join(User, Report.owner_id == User.id).filter(User.gender == 1).all()
    外连接 outerjoin
    使用 join 后就只能使用 filter() 特别指定用哪个模型的哪个字段来搜索

```

执行顺序

```text
GROUP BY子句必须出现在WHERE子句之后，ORDER BY子句之前。
HAVING语句必须在ORDER BY子句之后。

where -> group by -> order by -> having
```

## 二

```python
from apps.models import *

# 查找 手机号以 7 结尾的用户
User.query.filter(User.cellphone.like('%7')).all()

```
