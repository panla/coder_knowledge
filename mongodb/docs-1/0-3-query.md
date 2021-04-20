# 文档查询

## base

```text
# {item: "journal", qty: 25, status: "A", size: {h: 14, w: 21, uom: "cm"}, tags: ["blank", "red"]},
# {item: "notebook", qty: 50, status: "A", size: {h: 8.5, w: 11, uom: "in"}, tags: ["red", "blank"]},
# {item: "paper", qty: 10, status: "D", size: {h: 8.5, w: 11, uom: "in"}, tags: ["red", "blank", "plain"]},
# {item: "planner", qty: 0, status: "D", size: {h: 22.85, w: 30, uom: "cm"}, tags: ["blank", "red"]},
# {item: "postcard", qty: 45, status: "A", size: {h: 10, w: 15.25, uom: "cm"}, tags: ["blue"]}
```

## 基本语法

```text
# 格式化
.pretty()

# 查询数据
db.students.find({})

# 查询符合条件的第一条
db.students.findOne({})

```

## 查询语法

```text
# 根据 name 查询
db.students.find({name: 'panla'})

# 根据 name age 联合查询
db.students.find({name: 'panla', age: 17})

# size 是字典，查找，size: uom = in
db.inventory.find({"size.uom": "in"})

# tags 是数组，查找数组中有 red 的
db.inventory.find({tags: "red"})

$gt $gte
    大于，大于或等于
    {age: {$gt: 1}}
$lt $lte
    小于，小于或等于
    {age: {$lt: 1}}
$in $nin
    in not
    {age: {$in: [15, 18]}}
$exists
    {is_delete: {$exists: false}}
$ne <> !=
    {age: {$ne: 1}}

user.score
    嵌套
$elemMatch
    匹配数组内元素
$size
    匹配数组大小
$not
    取反
$all
    匹配所有
$slice

$where
    js
```