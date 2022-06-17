# 简单操作

[toc]

## auth

```text
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

```text
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

```text
db.collection.xxx

show collstions
db.createCollection(name, options)
db.students.drop()
```

### index

```text
创建索引
db.students.ensureIndex({name: 1}, {unique: false})
```

## document

### 数据类型

```text
string 字符串 UTF-8

integer 整数
long 64 位整数
double 双精度，浮点值
decimal

boolean 布尔
null 空值

timestamp 时间戳
date 日期，以UNIX存储 64位整数，代表自Unix纪元（1970年1月1日）以来的毫秒数

array 数组，把数组/列表/多个值存入一个键中
binary data 二进制数据

object 对象，嵌入式文档
objectid

max min keys 最大最小键，将值与最低最高BSON元素进行比较
symbol
code
re
```

### 数据类型2

```text

_id
    在MongoDB中，存储在集合中的每个文档都需要一个唯一的 _id字段作为主键

    默认自动创建，ObjectId 类型，12个字节组成，时间戳（秒，4字节），机器唯一标识（3字节），进程ID（2字节），随机数（3字节）

点符号
    MongoDB使用点符号访问数组的元素并访问嵌入式文档的字段。

    数组
        tags.2

    嵌入式文档
        "<embedded document>.<field>"

单个文档限制 16MB

查询过滤
    field: value

date
    new Date()
    ISODate()

    .toString()
    .getMonth()

```
