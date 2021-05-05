# Field

[Fields](https://tortoise-orm.readthedocs.io/en/latest/fields.html)

Fields are defined as properties of a Model class object
字段被定义为模型类对象的属性

## 参数

```text
source_filed str  设置数据库中的实际值
pk           bool 主键
null         bool 是否允许为空
default      Any  默认值
unique       bool 是否唯一
index        bool 是否索引
description  str  描述，comment
generated    bool
model
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

ManyToManyField
OneToOneField
```
