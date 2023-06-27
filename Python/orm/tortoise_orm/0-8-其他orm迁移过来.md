# 更换orm至tortoise-orm

[TOC]

## 按照原先的表的字段，整理tortoise-orm模型

```text
第一种，根据表的 SQL 语句，转换成新的 tortoise-orm 模型
第二种，根据原有的 ORM 模型，转换成新的 tortoise-orm 模型
需要完整地转换，
字段
    字段名
    类型
    非空
    默认
    注释
    顺序
约束与索引
    索引
    联合索引
    唯一
    联合唯一
    外键
```

## 增加 aerich 表

example

`migrations/models/0_20210727172655_init.sql`

```sql
-- upgrade --
CREATE TABLE IF NOT EXISTS `aerich` (
    `id` INT NOT NULL PRIMARY KEY AUTO_INCREMENT,
    `version` VARCHAR(255) NOT NULL,
    `app` VARCHAR(20) NOT NULL,
    `content` JSON NOT NULL
) CHARACTER SET utf8mb4;

```

## 更新迁移

```bash
aerich upgrade
```

## 进行新的迁移操作

pass
