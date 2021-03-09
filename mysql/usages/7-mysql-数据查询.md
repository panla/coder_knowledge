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
