# search 2

[TOC]

## collapse

```json
{
  "query": {
    "match_phrase": {
      "name": "人参"
    }
  },
  "collapse": {
    "field": "id"
  }
}
```

产生了一个 **fields**

```json
{
    "_index" : "remote",
    "_type" : "medicines",
    "_id" : "349",
    "_score" : 8.621462,
    "_source" : {
        "id" : 349,
        "is_delete" : false,
        "name" : "人参叶",
        "price" : 0.39,
        "price_unit" : "g",
        "pharmacy_id" : 1,
        "stock" : 100000,
        "aliases" : null,
        "desc" : null,
        "verify_info" : null,
        "status" : 3
    },
    "fields" : {
        "id" : [
        349
        ]
    }
}
```

## hightlight

pass

## 异步查询

`POST /example/_async_search`

[文档](https://www.elastic.co/guide/en/elasticsearch/reference/current/async-search.html#submit-async-search)
