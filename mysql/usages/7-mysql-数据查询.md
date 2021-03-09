# MySQL 数据查询

## 一般查询

```sql
select * from `students`;
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

## 排序

```sql
select * from `students` order by `id`;
```

## 分组

## 与 sqlalchemy 结合

```text
# 根据条件查询
select name from students where id = 1;
Student.query.filter_by(id=1)
Student.query.filter(Student.id == 1)

# 连接查询
select students.id, students.name, students.grade_id, grades.name from students inner join grades on students.grade_id = grades.id where students.id = 1;
Student.query.join(Grade, Student.grade_id == Grade.id).filter(Student.id == 1)

# >
select name from students where id > 10;
Student.query.filter(Student.id > 0)

# like 模糊匹配
select * from students where name like '%高%';
Student.query.filter(Student.name.contains('高'))
Student.query.filter(Student.name.like('%高%'))

# is
select * from students where name is not null;
Student.query.filter(Student.name.isnot(None))

select * from students where name is null;
Student.query.filter(Student.name.is_(None))

# in 范围查询
select * from students where name in ('a', 'b', 'c');
Student.query.filter(Student.name.in_(['a', 'b', 'c']))
Student.query.filter(Student.name.notin_(['a', 'b', 'c']))
```
