# SEARCH

默认情况下，hits响应部分包括与搜索条件匹配的前10个文档

## _search

`POST/GET`

`GET /example_a/medicines/_search?q=*&sort=id:asc`

```text
q=*是查询所
sort=id:asc 是按照id排序
```

## REQUEST BODY

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

## 分页, from, size

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

## 全文检索, match

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

## 短语匹配, `match_phrase`

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

## 多字段匹配，`multi_match`

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

## 完全匹配 term

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

## 复合查询bool

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

## 过滤查询 filter，区间查询操作

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

## keyword

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

## 分组查询

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
