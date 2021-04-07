# Field Mapping

迷迷糊糊

## 指定字段类型

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

### text 和 keyword

text 会分词然后索引，用于全文索引，支持聚合
keyword 不进行分词直接索引，用于关键词搜索，不支持聚合
支持模糊，精确查询

如果不指定类型，ElasticSearch字符串将默认被同时映射成text和keyword类型，会自动创建下面的动态映射
一个字符串字段可以映射为text字段用于全文本搜索，也可以映射为keyword字段用于排序或聚合

```json
{
    "alies" : {
        "type" : "text",
        "fields" : {
            "keyword" : {
                "type" : "keyword",
                "ignore_above" : 256
            }
        }
    }
}
```

## Dynamic

| JSON data type | "dynamic": "true" | "dynamic": "runtime" |
| :-: | :-: | :-: |
| null | No field added | No field added |
| true or false | boolean | boolean |
| double | float | float |
| integer | long | long |
| object | object | object |
| array | 取决于第一个不为 null 的值的类型 | 取决于第一个不为 null 的值的类型 |
| date string | date | date |
| numeric string | float or long | double or long |
| string | 带有 .keyword 的 text | keyword |

### 1 example

```json
{
  "id": 1,
  "name": "人参",
  "price": 0.91,
  "is_delete": false,
  "alies": ["萝卜干", "地下萝卜"],
  "created_at": "2021-1-1",
  "updated_at": "2021-1-1 10:10:10",
  "veridy_info": null
}
```

自动产生的 map, （不太准确）

```json
{
    "properties" : {
        "alies" : {
            "type" : "text",
            "fields" : {
                "keyword" : {
                    "type" : "keyword",
                    "ignore_above" : 256
                }
            }
        },
        "id" : {
            "type" : "long"
        },
        "is_delete" : {
            "type" : "boolean"
        },
        "name" : {
            "type" : "text",
            "fields" : {
                "keyword" : {
                    "type" : "keyword",
                    "ignore_above" : 256
                }
            }
        },
        "price" : {
            "type" : "float"
        },
        "created_at" : {
            "type" : "text",
            "fields" : {
                "keyword" : {
                    "type" : "keyword",
                    "ignore_above" : 256
                }
            }
        },
        "updated_at" : {
            "type" : "text",
            "fields" : {
                "keyword" : {
                    "type" : "keyword",
                    "ignore_above" : 256
                }
            }
        }
    }
}
```
