# MySQL DML

## insert

```sql
INSERT INTO `students` (`name`) VALUES ('王一');
INSERT INTO `students` (`name`) VALUES ('王二'), ('王三');
INSERT INTO `students` (`id`, 'name') VALUES (4, '王四');
```

## UPDATE

```sql
UPDATE `students` SET `name` = '大佬' WHERE `name` = '小弟';
UPDATE `students` SET `name` = '大佬', `email` = 'aaa@163.com' WHERE `name` = '小弟';
-- 闭合区间
UPDATE `students` SET `name` = '大佬' WHERE `id` BETWEEN 1 AND 10;

-- 字符替换
UPDATE `students` SET `name` = REPLACE(`name`, '小', '大') WHERE name LIKE '%小%';
```

## delete

```sql
DELETE FROM `students`;
DELETE FROM `students` WHERE id = 1;
```
