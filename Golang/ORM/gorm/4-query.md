# Query

[toc]

## doc

<https://gorm.io/zh_CN/docs/query.html>

## 检索单个对象

```go
// 获取第一条记录
// select * from users order by id limit 1;
db.First(&user)

// 获取一条记录，无排序
// select * from users limit 1;
db.Take(&user)

// 获取最后一条记录
// select * from users order by id desc limit 1;
db.Last(&user)

result := db.First(&user)
result.RowsAffected // 返回记录数
result.Error        //

errors.Is(result.Error, gorm.ErrRecordNotFound)

// find
db.Limit(1).Find(&user)
```

## 检索全部对象

## 条件

## 选择特定字段

## 排序

## Limit Offset

## Group By Having

## Distinct

## Joins

## Scan
