# MySQL 数据查询

```sql
SELECT *
FROM tb_name 
INNER JOIN tb_name ON tb_name.pid = tb_name.id 
WHERE ...
GROUP BY ...
HAVING ...
ORDER BY ...
LIMIT ...
```

## 一般查询

需要根据某些条件查询指定表里的指定数据

```sql
SELECT * FROM `students`;
SELECT `name` FROM `students`;
SELECT `name` FROM `students` WHERE id = 1;
SELECT `name` AS '姓名' FROM `students`;
SELECT CONCAT('姓名：', `name`) FROM `students`;
SELECT COUNT(`id`) FROM `students`;
SELECT DISTINCT `name` FROM `students`;
```

```sql
SELECT * FROM `results` WHERE score >= 95 AND score <= 100;
SELECT * FROM `results` WHERE score >= 95 && score <= 100;
SELECT * FROM `results` WHERE BETWEN 95 AND 100;
```

## 连接查询

```sql
SELECT `students`.id, `students`.name, `grades`.id, `grades`.name FROM `students` INNER JOIN `grades` on `students`.grade_id = `grades`.id;
```

自连接

```sql
SELECT a.name AS '父栏目', b.name AS '子栏目' FROM `categories` a, `categories` b WHERE a.id = b.pid;
```

## 分组

```sql
select name, avg(score), max(score), min(score) from results group by id;
```

## 排序

```sql
SELECT `grade_id`, COUNT(id) AS '该年级的学生数量' FROM `students` ORDER BY `grade_id`;
-- DESC 降序
-- ASC 升序，默认
```

## 分页

```sql
-- 起始值，页面大小
-- 1, 5
limit 0, 5

-- 2, 6
limit 1, 5

limit 0, 5
limit 5, 5
limit 10, 5
```

## 与 sqlalchemy 结合

```text
# 根据条件查询
SELECT name FROM students WHERE id = 1;
Student.query.filter_by(id=1)
Student.query.filter(Student.id == 1)

# 连接查询
SELECT students.id, students.name, students.grade_id, grades.name FROM students INNER JOIN grades ON students.grade_id = grades.id WHERE students.id = 1;
Student.query.join(Grade, Student.grade_id == Grade.id).filter(Student.id == 1)

# >
SELECT name FROM students WHERE id > 10;
Student.query.filter(Student.id > 0)

# like 模糊匹配
SELECT * FROM students WHERE name LIKE '%高%';
Student.query.filter(Student.name.contains('高'))
Student.query.filter(Student.name.like('%高%'))

# is
SELECT * FROM students WHERE name IS NOT NULL;
Student.query.filter(Student.name.isnot(None))

SELECT * FROM students WHERE name IS NULL;
Student.query.filter(Student.name.is_(None))

# in 范围查询
SELECT * FROM students WHERE name IN ('a', 'b', 'c');
Student.query.filter(Student.name.in_(['a', 'b', 'c']))
Student.query.filter(Student.name.notin_(['a', 'b', 'c']))
```
