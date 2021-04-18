# 简单操作

## auth

```bash
use admin
db.createUser( {user: "admin",pwd: "123456",roles: [ { role: "userAdminAnyDatabase", db: "admin" } ]})

db.auth(user, passwd)
```

```text
数据库用户角色
    read：只读
    readWriter：读写
数据库管理角色：
    dbAdmin：允许用户在指定的数据库中执行管理函数，创建索引，删除查看统计
    dbOwner：
    userAdmin：允许用户向system.usera集合写入，可以创建删除管理用户
集群管理角色：clusterAdmin clusterManager clusterMonitor hostManager
备份恢复角色：backup restore
所有数据库角色：
    readAnyDatabase： 授予用户所有数据库读权限
    readWriterAnyDatabase： 授予用户所有数据库读写权限
    userAdminAnyDatabase：
    dbAdminAnyDatabase
超级用户角色：root
```

## db

```bash
# 返回当前所在数据库
db

# 返回所有数据库
show dbs

# 切换，创建数据库
use example

# 删除
use example
db.dropDatabase
```

## collection

```bash
db.collection.xxx

show collstions
db.createCollection(name, options)
db.students.drop()
```

### document

#### insert

```bash
# 向 students 插入数据
db.students.insert([{name: "panla", age: 17, gender: 1}, {name: "pandora", age: 16, gender: 0}])

db.students.insertOne({})
db.students.insertMany([{name: "panla", age: 17, gender: 1}, {name: "pandora", age: 16, gender: 0}])

# 相同_id ObjectId 更新插入
db.students.save([{name: "panla", age: 17, gender: 1}])
```

#### update

```bash
db.students.update({_id: ObjectId("")}, {age: 12})
db.students.update({_id: ObjectId("")}, {age: 12})
```

#### query

```bash
# 格式化
.pretty()

# 查询数据
db.students.find({})
# { "_id" : ObjectId("607937987d1414fc08c932bd"), "name" : "panla", "age" : 17, "gender" : 1 }
# { "_id" : ObjectId("607937987d1414fc08c932be"), "name" : "pandora", "age" : 16, "gender" : 0 }

db.students.findOne({})

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
