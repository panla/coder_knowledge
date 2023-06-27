# 特别参数

[TOC]

## _update

POST 带上 _update，会先对比，再决定是否更新

推荐POST，PUT 需全字段更新

```text
POST /exmaple_a/medicines/1/_update
```

## _bulk 批量处理

```text
POST /exmaple_a/medicines/_bulk
{"index": {"_id": "Ohe1pXgBBdVu-MYeRhgi"}}
{"name": "人参叶"}
{"index": {"_id": "Ohe1pXgBBdVu-abcdefg"}}
{"name": "人参茎"}
```
