# MySQL语法

## 常用命令-1

```sql
-- 查看创建数据库的语句
SHOW CREATE DATABASE schools;

-- 查看创建表的语句
SHOW CREATE table students;

-- 查看表结构
DESC students;
```

## 常用命令-2

## DDL 数据定义

### 数据库与表

```sql
-- 修改表名
ALTER TABLE students RENAME AS new_students;

-- 添加字段
ALTER TABLE students ADD age int(11) DEFAULT 20;

-- 修改字段
ALTER TABLE students MODIFY name VARCHAR(20);

-- 修改字段名
ALTER TABLE students CHANGE name name1 VARCHAR(20);

-- 删除字段
ALTER TABLE students DROP name1;

ALTER TABLE students COMMENT '学生信息表';
```

### 外键

有好处，有坏处。

```sql
-- 第一种方案，建表时建外键
KEY `fk_grade_id` (`grade_id`),
CONSTRAINT `fk_grade_id` FOREIGN KEY (`grade_id`) REFERENCES `grade` (`id`)

-- 第二种方案
ALTER TABLE `students` ADD CONSTRAINT `fk_grade_id` FOREIGN KEY (`grade_id`) REFERENCES `grade` (`id`);
```

### 索引与约束

MySQL 中唯一约束是通过唯一索引实现的，为了保证没有重复值，在插入新记录时会再检索一遍，为了快速检索数据，在创建唯一约束的时候就创建了唯一索引。

```sql
CREATE TABLE `students` (
  `id` int NOT NULL AUTO_INCREMENT,
  `name` VARCHAR(30) COLLATE utf8mb4_general_ci NOT NULL COMMENT '姓名',
  `account_id` int NOT NULL COMMENT '账号id',
  `phone` VARCHAR(30) COLLATE utf8mb4_general_ci NOT NULL COMMENT '手机号',
  PRIMARY KEY (`id`),
  UNIQUE KEY `name` (`name`),
  UNIQUE KEY `ix_students_phone` (`phone`),
  KEY `ix_students_account_id` (`account_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci
```

结合 sqlalchemy

```python
name = db.Column(db.String(30), nullable=False, unique=True, comment='姓名')
account_id = db.Column(db.Integer, nullable=False, index=True, comment='账号id')
phone = db.Column(db.String(30), nullable=False, index=True, unique=True, comment='手机号')
```

```text
name        字段只写了  unique          结果生成了 唯一约束，在约束和索引里都可以找到。
account_id  字段只写了  index           结果生成了 普通索引，只在索引里找到。
phone       字段写了    index unique    结果生成了 唯一索引，在约束和索引里都可以找到。(此处与PGSQL有所不同)
```
