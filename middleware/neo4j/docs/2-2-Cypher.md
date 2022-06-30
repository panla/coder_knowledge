# Cypher

<https://neo4j.com/docs/cypher-manual/current/>

## 1 值和数据类型

属性类型

```text
Integer, Float
String, Boolean
Point
Date, Time, LocalTime, DateTime, LocalDateTime, Duration
```

结构类型

```text
Node 节点
    Id
    Label, Labels

    VehicleOwner
Relation 关系
    Id
    Type
    Map of properties
    Id for the start node
    Id for the end node

    OWNS_VEHICLE
Path 路径
    节点和关系的交替序列
```

复合类型

```text
列表，Map
```

## 3 表达式

### 3.1 CASE

```Cypher
CASE test
    WHEN value THEN result
    [...]
    [ELSE default]
END

MATCH (n)
RETURN
CASE n.eyes
  WHEN 'blue'  THEN 1
  WHEN 'brown' THEN 2
  ELSE 3
END AS result
```

## 4 变量

```Cypher
MATCH (n)-->(b)
RETURN b
```

## 5 保留关键字

<https://neo4j.com/docs/cypher-manual/current/syntax/reserved/>

