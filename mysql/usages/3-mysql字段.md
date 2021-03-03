# MySQL 字段

## 不同字段的长度大小范围

| column | 字段 | 长度大小范围 | 长度大小范围2 |
| :-: | :-: | :-: | :-: |
| tinyint | 最小整型 一个字节 |  -128 -- 127 | -2^7 -- 2^7 - 1 |
| tinyint unsigned | 最小整型 一个字节 |  0 -- 255 | 0 -- 2^8 - 1 |
| smallint | 小整型 两个字节 |  -32768 -- 32767 | 0 -- -2^15 -- 2^15 - 1 |
| smallint unsigned | 小整型 两个字节 |  0 -- 65535 | 0 -- 2^16 - 1 |
| mediumint | 整型 三个字节 | -8388608 -- 8388607 | -2^23 -- 2^23 -- 1 |
| mediumint unsigned | 整型 三个字节 | 0 -- 16777215 | 0 -- 2^24 -- 1 |
| int | 整型 四个字节 | -2147483648 -- 2147483647 | -2^31 -- 2^31 - 1 |
| int unsigned  | 整型 四个字节 | 0 -- 4294967295 | 0 -- 2^32 - 1 |
| bigint | big整型 八个字节 | -9223372036854775808 -- 9223372036854775807 | -2^63 -- 2^63-1 |
| bigint unsigned | big整型 八个字节 | 0 -- 18446744073709551615 | 0 -- 2^64 - 1 |
| varchar | 变长字符串 |  |  |
| float | 浮点数，四个字节 |  |  |
| double | 浮点数，八个字节，不精准 |  |  |

## 字符串

| column | 字段 | 长度大小范围 |
| :-: | :-: | :-: | :-: |
| char | 固定大小 | 0 -- 255 |
| varchar | 变长，2字节 | 0 -- 65535 |
| tinytext | 微型文本 | 0 -- 255 |
| text | 文本串 | 2字节 | 0 -- 65535 |

## 时间日期

| column | 字段 | 其他 |
| :-: | :-: | :-: | :-: |
| date | 日期 | |
| time | 时间 | |
| datetime | 日期时间 | |
| timestample | 时间戳 | 2038 |
| year | 年份 | |

## 数据库字段属性

### 主键

```sql
PRIMARY KEY (`id`)
```

### 唯一

```sql
UNIQUE KEY `cellphone` (`cellphone`)
```

### 索引

```sql
KEY `ix_accounts_name` (`name`),
```

### 非空

NOT NULL

### 无符号

unsigned 不可为负数

### 自增

auto_increment
AUTO_INCREMENT

### 默认值

```sql
DEFAULT '0'
```

### 注释

comment '手机号'

### 更新

```sql
ON UPDATE
```

## sqlalchemy 中选择无符号类型

第一种，只能在 mysql 中使用

```python
from sqlalchemy.dialects.mysql import INTEGER

# unsigned=True

db.Column(INTEGER(unsigned=True), comment='无符号整型')

```

第二种，自定义无符号，可以在 mysql/sqlite3 种使用

```python
from sqlalchemy import Integer
from sqlalchemy.dialects.mysql import INTEGER

UnsignedInt = Integer()
UnsignedInt = UnsignedInt.with_variant(INTEGER(unsigned=True), 'mysql')

db.Column(UnsignedInt, comment='无符号整型')
```

备注

```text
对于上面第二种，存疑

在 sqlite 中确实可以向 无符号整型字段 存储 9223372036854775807, 对于明显超过了 整型的范围的数字也能存进去。。
```
