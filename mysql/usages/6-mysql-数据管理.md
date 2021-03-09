# MySQL DML

## insert

```sql
insert into `students` (`name`) values ('王一');
insert into `students` (`name`) values ('王二'), ('王三');
insert into `students` (`id`, 'name') values (4, '王四');
```

## update

```sql
update `students` set `name` = '大佬' where `name` = '小弟';
update `students` set `name` = '大佬', `email` = 'aaa@163.com' where `name` = '小弟';
-- 闭合区间
update `students` set `name` = '大佬' where `id` between 1 and 10;

```

## delete

```sql
delete from `students`;
delete from `students` where id = 1;
```
