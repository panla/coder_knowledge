# 基础API使用

[参考一](https://www.cnblogs.com/balloon72/p/13177872.html)

## es 数据

```text
# 删除索引
DELETE /example_a*

# 查看所有索引
GET /_cat/indices?v

# 查看已经安装的插件
GET /_cat/plugins

# 查看节点信息
GET /_cat/nodes
GET /_cat/nodes?v
GET /_cat/allocation?v

# 查看分片信息
GET /_cat/shards?v
```

### 指定字段类型

```text
PUT /medicines
{
  "mappings": {
    "properties": {
      "name": {
        "type": "text"
      },
      "age": {
        "type": "date"
      }
    }
  }
}
```

## PUT

### 更新一条药材数据

创建或更新数据，须指定文档的索引名称，唯一的文档 ID以及数据

```text
PUT /medicines/_doc/1
{
  "name": "人参",
  "price": 0.08,
  "is_delete": false
}
```

response

```json
{
  "_index" : "medicines",
  "_type" : "_doc",
  "_id" : "1",
  "_version" : 2,
  "result" : "updated",
  "_shards" : {
    "total" : 2,
    "successful" : 1,
    "failed" : 0
  },
  "_seq_no" : 2,
  "_primary_term" : 1
}
```

## POST

### 创建一条药材数据

保存在指定的索引的类型下，可以指定唯一标识，不指定时则会自动生成

```text
POST /medicines/_doc
{
  "id": 1,
  "name": "人参",
  "price": 0.07,
  "is_delete": false
}
```

response

```json
{
  "_index" : "medicines",
  "_type" : "_doc",
  "_id" : "Ohe1pXgBBdVu-MYeRhgi",
  "_version" : 1,
  "result" : "created",
  "_shards" : {
    "total" : 2,
    "successful" : 1,
    "failed" : 0
  },
  "_seq_no" : 0,
  "_primary_term" : 1
}
```

```text
POST /medicines/_doc/1
{
  "id": 1,
  "name": "人参",
  "price": 0.07,
  "is_delete": false
}
```

### 更新部分字段数据

```text
POST /medicines/_doc/1/_update
```

## GET

### 查看一条药材详情

```text
GET /example_a/medicines/1
```

response

```json
{
  "_index" : "medicines",
  "_type" : "_doc",
  "_id" : "1",
  "_version" : 1,
  "_seq_no" : 1,
  "_primary_term" : 1,
  "found" : true,
  "_source" : {
    "name" : "人参",
    "price" : 0.08,
    "is_delete" : false
  }
}
```

## DELETE

### 删除一条药材数据

```text
DELETE /medicines/_doc/1
```
