# 文档查询

[toc]

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

## 聚合

```text
$sum
%max
$min
$avg
$push       将值插入到一个结果文档的数组中
$addToSet   将值插入到一个结果文档的数组中，但不进行复制
$first      根据成组方式，从源文档中获取第一个文档。但只有对之前应用过 $sort 管道操作符的结果才有意义。
$last       根据成组方式，从源文档中获取最后一个文档。但只有对之前进行过 $sort 管道操作符的结果才有意义。
```

## 管道

```text
$project
$match
$group
$sort
$skip
$limit
$unwind
```

### 排序

```text
升序
sort({key: 1})

降序
sort({key: -1})
```

### sum example

```text
{ "_id" : ObjectId("60827aa55a578bb6aaa35701"), "name" : "4", "age" : 16, "created_at" : ISODate("2021-04-23T07:43:33.661Z") }
{ "_id" : ObjectId("60827aac5a578bb6aaa35702"), "name" : "5", "age" : 13, "created_at" : ISODate("2021-04-23T07:43:40.966Z") }
{ "_id" : ObjectId("607e9d92c73e72a912c801fa"), "name" : "2", "age" : 12, "created_at" : Timestamp(1618910610, 1) }
{ "_id" : ObjectId("607e9ea5c73e72a912c801fb"), "name" : "3", "age" : 12, "created_at" : ISODate("2021-04-20T09:28:05.118Z") }

db.mycol.aggregate([{$group : {_id : "$age", num : {$sum : 1}}}])

{ "_id" : 13, "num" : 1 }
{ "_id" : 12, "num" : 2 }
{ "_id" : 16, "num" : 1 }
```
