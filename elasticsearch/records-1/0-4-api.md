# 基础API使用

[参考一](https://www.cnblogs.com/balloon72/p/13177872.html)

## es 数据

### 删除索引

```text
DELETE /example_a*
```

### 查看所有索引

```text
GET /_cat/indices
```

## POST

- `example_a.medicines`

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

## 额外参数

### _update

POST 带上 _update，会先对比，再决定是否更新

```text
POST /exmaple_a/medicines/1/_update
```

### _bulk 批量处理

```text
POST /exmaple_a/medicines/_bulk
{"index": {"_id": "Ohe1pXgBBdVu-MYeRhgi"}}
{"name": "人参叶"}
{"index": {"_id": "Ohe1pXgBBdVu-abcdefg"}}
{"name": "人参茎"}
```

## SEARCH

默认情况下，hits响应部分包括与搜索条件匹配的前10个文档

### _search

`GET /example_a/medicines/_search?q=*&sort=id:asc`

```text
q=*是查询所
sort=id:asc 是按照id排序
```

### REQUEST BODY

`GET /example_a/_search`

```json
{
    "query": {
        "match_all": {}
    },
    "sort": [
        {
            "id": "asc"
        }
    ]
}
```

注意：**如果在搜索中指定 type 会提示 Specifying types in search requests is deprecated.**

### 分页, from, size

`GET /example_a/_search`

```json
{
    "query": {
        "match_all": {}
    },
    "sort": [
        {
            "id": "asc"
        }
    ],
    "from": 0,
    "size": 2,
    "_source": []
}
```

`_source` 返回指定的参数

### 全文检索, match

`GET /example_a/_search`

```json
{
    "query": {
        "match": {
            "name": "人"
        }
    }
}
```

### 短语匹配, `match_phrase`

有的人或文档翻译解释为精准匹配，很奇怪

`GET /example_a/_search`

```json
{
    "query": {
        "match_phrase": {
            "name": "人参"
        }
    }
}
```

### 多字段匹配，`multi_match`

涉及到匹配评分的问题

`GET /example_a/_search`

```json
{
    "query": {
        "multi_match": {
            "query": "",
            "fields": ["name", "email"]
        }
    }
}
```

匹配方案, type

```text
best_fields 完全匹配，"tie_breaker": 0.3，完全匹配的评分考前，不能的评分乘以0.3的系数
most_fields 最多字段匹配，
cross_fields 词条的分词词汇是分配到不同字段中
```

### 完全匹配 term

完全匹配，即不进行分词器分析，文档中必须包含整个搜索的词汇

`GET /example_a/_search`

```json
{
    "query": {
        "term": {
            "content": "人参"
        }
    }
}
```

### 复合查询bool

- must: 必须满足
- must_not: 必须不满足
- should: 应该满足, 不过不满足的也能查出来，得分低
- range: 区间查询

```json
{
  "query": {
    "bool": {
      "must": [
        {
            "match": {
                "gender": "F"
                }
        },
        {
            "match": {
                "address": "Mill"
                }
        }
      ],
      "must_not": [
        {
            "match": {
                "age": "38"
                }
        }
      ],
      "should": [
        {
            "match": {
                "lastname": "Long"
            }
        }
      ]
    }
  }
}
```

### 过滤查询 filter，区间查询操作

```json
{
    "query": {
        "bool": {
            "filter": {
                "range": {
                    "price": {
                        "gte": 0.05,
                        "lte": 1
                    }
                }
            }
        }
    }
}
```

### keyword

有 keyword 时精确查找，没有时，会成为关键字?????

```json
{
    "query": {
        "match": {
            "address.keyword": "人参"
        }
    }
}
```

### 分组查询

```json
{
    "id" : 4,
    "user_id" : 2,
    "price" : 120000,
    "brand" : "奔驰"
}
```

注意：**数字类型和字符串类型的 查询field不同**

以 `user_id` 分组查询平均价格

```json
{
  "aggs": {
    "group_by_user_id": {
      "terms": {
        "field": "user_id"
      },
      "aggs": {
        "average_price": {
          "avg": {
            "field": "price"
          }
        }
      }
    }
  }
}
```

以 brand 分组查询平均价格

```json
{
  "aggs": {
    "group_by_brand": {
      "terms": {
        "field": "brand.keyword"
      },
      "aggs": {
        "average_price": {
          "avg": {
            "field": "price"
          }
        }
      }
    }
  }
}
```
