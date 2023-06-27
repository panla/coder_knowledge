# Create

[TOC]

## doc

<https://gorm.io/zh_CN/docs/create.html>

## create record

```go
user := &User{
    Name: "pandora",
    Age:  18,
}

result *DB := db.Create(user)
```

## create record by columns

```go
// 用 user 的 Name 和 Age 创建一条记录
user := &User
db.Select("Name", "Age").Create(user)

// 创建一条记录，忽略 user.Name user.Age
db.Omit("Name", "Age").Create(user)
```

## batch insert

```go
users := []User{{Name: "pandora"}, {Name: "athena"}}

db.Create(&users)

// 此处用指针还是值
db.CreateInBatches(users, batch)
```

## 钩子

<https://gorm.io/zh_CN/docs/hooks.html>

### BeforeSave

### BeforeCreate

### AfterSave

### AfterCreate

## create by map

```go
db.Model(&User{}).Create(map[string]interface{}{
    Name: "pandora",
    Age:  18
})

db.Model(&User{}).Create([]map[string]interface{}{
    {Name: "pandora", Age:  18},
    {Name: "athena", Age:   12},
})
```

## 高级选项

### 关联创建

```go
type Address struct {
    gorm.Model
    Name string
}

type User struct {
    gorm.Model
    Name string
    Address Address
}

db.Create(&User{
    Name: "",
    Address: Address{
        Name: "",
    }
})
```

### 默认值

```text
`gorm:"default:pandora"`
```
