# ORM

[TOC]

## 字段类型以及参数

字符串类型

| 字段类型 | 说明 |
| :-: | :-: |
| CharField | 字符串 |
| TextField | 文本 |
| EmailField | 邮箱 |
| URLField | url |
| UUIDField | uuid |
| ChoiceField | 选择 |

数字类型

| 字段类型 | 范围 |
| :-: | :-: |
| IntegerField | -(2^31) ~~ 2^31 - 1 |
| SmallIntegerField | -(2^15) ~~ 2^15 - 1 |
| BigIntegerField | -(2 ^ 63) ~~ 2 ^ 63 - 1 |
| PositiveIntegerField | 0 ~~ 2 ^ 31 - 1 |
| PositiveSmallIntegerField | 0 ~~ 2 ^ 15 - 1 |
| AutoField | 1 ~~ 2^31 - 1 |
| SmallAutoField | 1 ~~ 2 ^ 15 - 1 |
| BigAutoField | 1 ~~ 2 ^ 63 - 1 |
| DecimalField | max_digits, decimal_places |

时间日期类型

| 字段类型 | 说明 |
| :-: | :-: |
| DateField | 日期,年月日 |
| TimeField | 时间,时分秒 |
| DateTimeField | 日期时间,年月日,时分秒 |

其他类型

| 字段类型 | 说明 |
| :-: | :-: |
| BooleanField | `bool` |
| ImageField | 图片,保存图片,存储图片路径 |
| FileField | 文件,保存文件,存储图片路径 |
| FilePathField |  |
| BinaryField | 二进制? |

一般参数

| 参数 | 说明 |
| :-: | :-: |
| max_length | CharField 所必需的参数 |
| default | 默认值 |
| null | `bool` 是否允许 null None |
| blank | `bool` 是否允许 空值 |
| db_index | `bool` 是否建立索引 |
| unique | `bool` 是否要求唯一 |
| choices | ((存储值, 显示值),) |
| verbose_name |  |
| help_text |  |

日期时间类型相关的参数

| 参数 | 说明 |
| :-: | :-: |
| auto_now | `bool` 是否是现在的值,一般用于更新时间 |
| auto_now_add | `bool` 是否是创建时时间,一般用与创建时间 |

## Meta

### 属性

| 属性 | 说明 |
| :-: | :-: |
| unique_together | `[[]]`, `[]` 联合唯一 |
| db_table | 数据库表名 |
| abstract | `bool` 是否是抽象类 |
| app_label | app 名称 |
| ordering | list 排序字段 |
| index_together | `[[]]`, `[]` 联合索引 |
| verbose_name |  |
| verbose_name_plunal |  |

## QuerySet

### 产生QuerySet

- all()
- filter()
- exclude()
- order_by()
- reverse()
- distinct()

### QuerySet function

- exists()
- first()
- last()
- count()
- update()
- all()
- filter()
- exlcude()
- order_by()
- reverse()
- distinct()

### filter

```text
filter(pub__date__gt=datetime.date(2020, 7, 7))
filter(name__contains='bob')
filter(Q(mobile=username) | Q(email=mobile))
filter(id__in=[1, 2, 3])
filter(id__range=[1, 3])
filter(name__startswith='A')
filter(name__endswith='Z')
```

## 外键关联

一对多

```text
book.publisher = publisher
books = publisher.books
```

多对多

```text
添加 book.authors.add(a1, a2)
创建 book.authors.create(**kwargs)
替换 book.authors.set([a1, a2])
移除 book.authors.remove(a1, a2)
清空 book.clear()
```
