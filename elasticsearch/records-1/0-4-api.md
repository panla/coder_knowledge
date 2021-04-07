# 基础API使用

[参考一](https://www.cnblogs.com/balloon72/p/13177872.html)

## es 数据

```text
# 删除索引
DELETE /example_a*

# 查看所有索引
GET /_cat/indices

# 查看已经安装的插件
GET /_cat/plugins
```

### 指定字段类型

```text
PUT /example_a
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

## POST

### 创建一条药材数据

保存在指定的索引的类型下，可以指定唯一标识，不指定时则会自动生成

```text
POST /example_a/medicines
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
  "_index" : "example_a",
  "_type" : "medicines",
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
POST /example_a/medicines/1
{
  "id": 1,
  "name": "人参",
  "price": 0.07,
  "is_delete": false
}
```

## PUT

### 更新一条药材数据

```text
PUT /example_a/medicines/1
{
  "name": "人参",
  "price": 0.08,
  "is_delete": false
}
```

response

```json
{
  "_index" : "example_a",
  "_type" : "medicines",
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

## GET

### 查看一条药材详情

```text
GET /example_a/medicines/1
```

response

```json
{
  "_index" : "example_a",
  "_type" : "medicines",
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
DELETE /example_a/medicines/1
```
