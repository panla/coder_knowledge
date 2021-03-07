# MySQL

DBMS：数据库管理系统，数据库的管理软件
关系型数据库管理系统

## SQL

- DDL: Dta Define Language 数据定义语言
- DML: Data Manipulation Language 数据操作语言
- DQL: Data Query Language 数据查询语言
- DCL: Data Control Language 数据控制语言

## 常用命令-1

```sql
-- 查看创建数据库的语句
show create database schools;

-- 查看创建表的语句
show create table students;

-- 查看表结构
desc students;
```

## 常用命令-2

```sql
-- 修改表名
alter table students rename as new_students;

-- 添加字段
alter table students add age int(11) default 20;

-- 修改字段
alter table students modify name varchar(20);

-- 修改字段名
alter table students change name name1 varchar(20);

-- 删除字段
alter table students drop name1;
```

## 常用命令3

```sql
create table sudents (
    `id` int(11) noy null auto_increment comment '主键',
    `name` varchar(30) noy null comment '名称',
    primary key (`id`),
    key `ix_students_name` (`name`)
) engine=innodb default charset=utf8mb4;
```

## 数据管理

### 外键

有好处，有坏处。

### insert

### update

### delete
