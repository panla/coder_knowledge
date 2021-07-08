# Field

[Fields](https://tortoise-orm.readthedocs.io/en/latest/fields.html)

Fields are defined as properties of a Model class object
字段被定义为模型类对象的属性

## 参数

```text
source_filed str  设置数据库中的实际值
pk           bool 主键
null         bool 是否允许为空 default=False
default      Any  默认值
unique       bool 是否唯一 default=False
index        bool 是否索引 default=False
description  str  描述，comment
generated    bool
model
validators   Optional[List[Union[Validator, Callable]]]
```

## data fields

```text
SmallIntField
IntField
BigIntField

DecimalField
    max_digits      int     该字段的最大有效位数
    decimal_places  int     小数点后有多少个有意义的数字
FloatField

CharField
    max_length
TextField

CharEnumField
    enum_type
    max_length
IntEnumField
    enum_type

TimeDeltaField
DateField
DatetimeField
    auto_now        bool    更新
    auto_now_add    bool    创建

JSONField, 尚不清楚是否支持 mysql5.6,5.7
    encoder
    decoder
    separators=(',', ':')   分割符
```

## 关系字段

```text
ForeignKeyField
    model_name
    related_name
    on_delete='CASCADE'
    db_constraint: bool = True,

ManyToManyField
    model_name: str,
    through: Optional[str] = None,
    forward_key: Optional[str] = None,
    backward_key: str = "",
    related_name: str = "",
    on_delete: str = CASCADE,
    db_constraint: bool = True,

OneToOneField
    model_name: str,
    related_name: Union[Optional[str], Literal[False]] = None,
    on_delete: str = CASCADE,
    db_constraint: bool = True,

```

## 自定义字段

```python
from typing import Any

from tortoise.fields.base import Field


class UnsignedTinyIntField(Field, int):
    SQL_TYPE = "TINYINT UNSIGNED"
    allows_generated = True

    def __init__(self, pk: bool = False, **kwargs: Any) -> None:
        if pk:
            kwargs["generated"] = bool(kwargs.get("generated", True))
        super().__init__(pk=pk, **kwargs)

    @property
    def constraints(self) -> dict:
        return {
            "ge": 1 if self.generated or self.reference else 0,
            "le": 255,
        }

    class _db_mysql:
        GENERATED_SQL = "TINYINT UNSIGNED NOT NULL PRIMARY KEY AUTO_INCREMENT"
```
