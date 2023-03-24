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

// find 避免 ErrRecordNotFound 错误
// 接受 struct or slice
// 对于单个对象使用 Find 且不使用 Limit 时，将查询全表返回第一个对象
db.Limit(1).Find(&user)

// db.Model
result := map[string]interface{}{}
db.Model(&User{}).First(&result)

db.Table("users").Take(&result)

// 没有主键时，将使用 第一个字段
```

### 根据主键检索

```go
db.First(&user, 10)
db.First(&user, "10")
// select * from users where id = 10;

db.Find(&users, []int{1, 2, 3})
// select * from users where id in (1, 2, 3);

// 主键是字符串
db.First(&user, "id = ?", "abcdefg")
// select * from users where id = "abcdefg";

user := User(ID: 10)
db.First(&user)

result := map[string]interface{}{}
db.Model(User(ID: 10)).First(&result)
// select * from users where id = 10;
```

## 检索全部对象

```go
result := db.Find(&users)

result.RowsAffected
result.Error
```

## 条件

### string 条件

```go
db.Where("name = ?", "pandora").First(&user)
// select * from users where name = "pandora";

db.Where("age > ?", 12).Find(&users)
// select * from users whereage > 12;

db.Where("name IN ?", []string{"a", "b"}).First(&users)
// select * from users where name IN ("a", "b");

db.Where("name LIKE ?", "%pan").First(&user)
// select * from users where name LIKE "%pan";
```

### struct & map 条件

```go
db.Where(&User{Name: "pandora"}).First(&user)
db.Where(&User{Name: "pandora"}).Find(&users)

db.Where(map[string]interface{}{"name": "pandora"}).Find(&users)

db.Where([]int64{20, 21, 22}).Find(&users)
```

当使用结构体查询时, 仅可查询非零值

### 指定结构体查询字段

```go
db.Where(&User{Name: "pandora"}, "name", "Age").Find(&users)
// select * from users where name = "pandora" and age = 0;

db.Where(&User{Name: "pandora"}, "Age").Find(&users)
// select * from users where age = 0;
```

### 内联条件

```go
// 非整型主键
db.First(&user, "id = ?", "abc")

// Plain SQL
db.Find(&user, "name = ?", "pandora")

db.Find(&users, "name <> ? and age > ?", "pandora", 20)
// select * from users where name <> "pandora" and age > 20;

// struct
db.Find(&users, User(Age: 20))

// Map
db.Find(&users, map[string]interface{}{"age": 20})
```

### not 条件

```go
db.Not("name = ?", "pandora").First(&user)
// SELECT * FROM users WHERE NOT name = "pandora" ORDER BY id LIMIT 1;

// not in
db.Not(map[string]interface{}{"name": []string{"a", "b"}}).Find(&users)
// select * from users where name not in ("a", "b");

db.Not([]int64{1, 2, 3}).First(&user)
// select * from users where id not in (1, 2, 3) order by id limit 1;
```

### or 条件

```go
db.Where("role = ?", "admin").Or("role = ?", "super_admin").Find(&users)
// select * from users where roll = "admin" or role = "super_admin";

db.Where("name = 'pandora'").Or(User{Name: "athena"}).Find(&users)
// select * from users where name = "pandora" or name = "athena";

db.Where("name = 'pandora'").Or(map[string]interface{}{"name": "athena"}).Find(&users)
```

## 选择特定字段

```go
db.Select("name", "age").Find(&users)

db.Select([]string{"name", "age"}).Find(&users)

db.Table("users").Select("COALESCE(age, ?)", 42).Rows()
```

## 排序

```go
db.Order("age desc, name").Find(&users)

db.Order("age desc").Order("name").Find(&users)
```

## Limit Offset

```go
db.Limit(3).Find(&users)

// 用 -1 取消 limit
db.Limit(10).Find(&users1).Limit(-1).Find(&users2)
// select * from users limit 10; (users1)
// select * from users; (users2)

db.Limit(10).Offset(5).Find(&users)
```

## Group By & Having

```go
db.Model(&User{}).Select("name, sum(age) as total").Where("name LIKE ?", "%pan").Group("name").First(&result)
// select name, sum(age) as total from `users` where name like "%pan" group by `name` limit 1;

db.Model(&User{}).Select("name, sum(age) as total").Group("name").Having("name = ?", "pandora").Find(&result)
// select name, sum(age) as total from `users` group by `name` having name = "pandora";

```

## Distinct

```go
db.Distinct("name", "age").Find(&users)
```

## Joins

```go
db.Model(&User{}).Select("users.name, emails.email").Joins("left join emails on emails.user_id = users.id").Scan(&result{})

db.Joins("JOIN emails ON emails.user_id = users.id AND emails.email = ?", "jinzhu@example.org").Joins("JOIN credit_cards ON credit_cards.user_id = users.id").Where("credit_cards.number = ?", "411111111111").Find(&user)
```

## Scan

```text
类似于 Find
```
