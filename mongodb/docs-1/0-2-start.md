# 简单操作

## db

```bash
# 返回当前所在数据库
db

# 返回所有数据库
show dbs

# 切换，创建数据库
use example
```

## collection

```bash
# 返回该数据库下的集合
show collections

db.collection.xxx
```

```bash
# 向 students 插入数据
db.students.insertMany([{name: "panla", age: 17, gender: 1}, {name: "pandora", age: 16, gender: 0}])

```

### 查询

```bash
# 查询数据
db.students.find({})
# { "_id" : ObjectId("607937987d1414fc08c932bd"), "name" : "panla", "age" : 17, "gender" : 1 }
# { "_id" : ObjectId("607937987d1414fc08c932be"), "name" : "pandora", "age" : 16, "gender" : 0 }

# 格式化
db.students.find({}).pretty()

# 根据 name 查询
db.students.find({name: 'panla'})

# 根据 name age 联合查询
db.students.find({name: 'panla', age: 17})

# {item: "journal", qty: 25, status: "A", size: {h: 14, w: 21, uom: "cm"}, tags: ["blank", "red"]},
# {item: "notebook", qty: 50, status: "A", size: {h: 8.5, w: 11, uom: "in"}, tags: ["red", "blank"]},
# {item: "paper", qty: 10, status: "D", size: {h: 8.5, w: 11, uom: "in"}, tags: ["red", "blank", "plain"]},
# {item: "planner", qty: 0, status: "D", size: {h: 22.85, w: 30, uom: "cm"}, tags: ["blank", "red"]},
# {item: "postcard", qty: 45, status: "A", size: {h: 10, w: 15.25, uom: "cm"}, tags: ["blue"]}

# size 是字典，查找，size: uom = in
db.inventory.find({"size.uom": "in"})
# {"_id": ObjectId("60793b927d1414fc08c932c0"),
# "item": "notebook", "qty": 50, "status": "A", "size": {"h": 8.5, "w": 11, "uom": "in"}, "tags": ["red", "blank"]}
# {"_id": ObjectId("60793b927d1414fc08c932c1"),
# "item": "paper", "qty": 10, "status": "D", "size": {"h": 8.5, "w": 11, "uom": "in"}, "tags": ["red", "blank", "plain"]}

# tags 是数组，查找数组中有 red 的
db.inventory.find({tags: "red"})

```
