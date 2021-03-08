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

```sql
-- 第一种方案，建表时建外键
key `fk_grade_id` (`grade_id`),
constraint `fk_grade_id` foreign key (`grade_id`) references `grade` (`id`)

-- 第二种方案
alter table `students` add constraint `fk_grade_id` foreign key (`grade_id`) references `grade` (`id`);
```

### insert

```sql
insert into `students` (`name`) values ('王一');
insert into `students` (`name`) values ('王二'), ('王三');
insert into `students` (`id`, 'name') values (4, '王四');
```

### update

```sql
update `students` set `name` = '大佬' where `name` = '小弟';
update `students` set `name` = '大佬', `email` = 'aaa@163.com' where `name` = '小弟';
-- 闭合区间
update `students` set `name` = '大佬' where `id` between 1 and 10;

```

### delete

```sql
delete from `students`;
delete from `students` where id = 1;
```

## DQL 数据查询

### 一般查询

```sql
from * from `students`;
select `name` from `students`;
select `name` from `students` where id = 1;
select `name` as '姓名' from `students`;
select concat('姓名：', `name`) from `students`;
select count(`id`) from `students`;
select distinct `name` from `students`;
```

```sql
select * from `results` where score >= 95 and score <= 100;
select * from `results` where score >= 95 && score <= 100;
select * from `results` where between 95 and 100;
```

### 排序

```sql
select * from `students` order by `id`;
```

### 分组
