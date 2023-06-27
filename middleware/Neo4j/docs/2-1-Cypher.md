# Cypher

[TOC]

## 1 Patterns

### 1.1 节点语法

使用 () 代表一个节点

```Cypher
()
(matrix)
(matrix:Movie)
(matrix:Movie {title: 'The Matrix'})
(matrix:Movie {title: 'The Matrix, released: 1999'})

# 变量      matrix
# 节点标签  :Movie
# 节点属性  {title: 'The Matrix'}
```

### 1.2 关系语法

```Cypher
-->
# 无向关系

-[role]->
-[:ACTION_IN]->
-[role:ACTED_IN]->
-[role:ACTED_IN {roles: ['Neo']}]->

# 源节点-[关系]->目标节点
```

### 1.3 模式语法

```Cypher
(keanu:Person:Actor {name: 'Keanu Reeves'})-[role:ACTED_IN {roles: ['Neo']}]->(matrix:Movie {title: 'The Matrix'})
电影：黑客帝国
演员：Keanu Reeves
```

### 1.4 模式变量

```Cypher
acted_in = (:Person)-[:ACTED_IN]->(:Movie)
```

## 2 Practice Patterns

### 2.1 创建 CREATE

```Cypher
# 创建 title=The Matrix, released=1997 的电影
# 节点：movie
# 标签：Movie
# 属性：title, released

CREATE(movie:Movie {title: 'The Matrix', released: 1997})
RETURN movie
```

### 2.2 匹配 MATCH

#### 2.2.1 特别的

清空删除节点

```Cypher
MATCH (n)
OPTIONAL MATCH (n)-[r]-()
DELETE n, r
```

查询所有

```Cypher
MATCH (p)
OPTIONAL MATCH (p:Person)-[r]-(m:Movie)
RETURN p, r, m
```

#### 2.2.2 MATCH

匹配查询 title=The Matrix 的电影

```Cypher
MATCH (movie:Movie {title: 'The Matrix'})
RETURN movie
```

查询 id=1的 point

```Cypher
MATCH (p:Point {id: 1})
RETURN p
```

查询 id=1的 point

```Cypher
MATCH (p:Point)
WHERE p.id = 1
RETURN p
```

查寻，id=1 的point 的下一个点

```Cypher
MATCH (p:Point {id: 1})-[r:NEXT]->(p2:Point)
RETURN  p, r, p2
```

#### 2.2.3 附加结构，查询，作为变量进行创建

```Cypher
MATCH (p:Person {name: 'Tom Hanks'})
CREATE (m:Movie {title: 'Cloud Atlas', released: 2012})
CREATE (p)-[r:ACTED_IN {roles: ['Zachry']}]->(m)
RETURN p, r, m
```

#### 2.2.4 完成模式 Completing patterns

幂等，查找或创建

```Cypher
# 查询 title='Cloud Atlas' 的 movie，如果存在，就设置 released = 2012，如果不存在就创建

MERGE (m:Movie {title: 'Cloud Atlas'})
ON CREATE SET m.released = 2012
RETURN m
```

## 3 correct results

### 3.2 过滤结果

WHERE combined with AND, OR, XOR, NOT, IN

=~

```Cypher
MATCH (m:Movie)
WHERE m.title = 'The Matrix'
RETURN m
```

```Cypher
MATCH (p:Person)-[r:ACTED_IN]->(m:Movie)
WHERE p.name =~ 'K.+' OR m.released > 2000 OR 'Neo' IN r.roles
RETURN p, r, m
```

### 3.3 return

### 3.4 汇总

```Cypher
MATCH (:Person)
RETURN count(*) AS people
```

### 3.5 排序，分页

```Cypher
MATCH (p:Person)-[:ACTED_IN]->(m:Movie)
RETURN p, count(*) AS num
ORDER BY num DESC LIMIT 10
```

### 3.6 收集聚合

```Cypher
MATCH (m:Movie)<-[:ACTED_IN]-(p:Person)
RETURN m.title AS movie, collect(p.name) AS cast, count(*) AS actors
```

## 4 较为复杂的语句

### 4.2 联合 UNION

同时列出导演和演员

```Cypher
MATCH (actor:Person)-[r:ACTED_IN]->(movie:Movie)
RETURN actor.name AS name, type(r) AS type, movie.title AS title
UNION
MATCH (director:Person)-[r:DIRECTED]->(movie:Movie)
RETURN director.name AS name, type(r) AS type, movie.title AS title
```

```Cypher
MATCH (p:Person)-[r:ACTED_IN|DIRECTED]->(movie:Movie)
RETURN p.name AS name, type(r) AS type, movie.title AS title
```

### 4.3 WITH

管道，每个片段都处理签一个片段的输出，其结果输入进下一个片段

```Cypher
MATCH (person:Person)-[:ACTED_IN]->(m:Movie)
WITH person, count(*) AS appearances, collect(m.title) AS movies
WHERE appearances > 1
RETURN person.name, appearances, movies
```

### 4.4 SET

更新

```Cypher
MATCH (n {name: 'pandora'})
SET n.name = 'athena'
RETURN n
```

### 4.5 DELETE

### 4.6 REMOVE

## 5 定义一个模式

### 5.2 使用索引

加速，提高性能

```Cypher
CREATE INDEX point_index_1 FOR (p:Point) ON (p.point_id)
CREATE INDEX point_index_1 FOR (p:Point) ON (p.uid)

CREATE INDEX point_index_1 FOR (p:Point) ON (p.point_id, p.name)

# 查看定义了哪些索引，索引名称，标签或类型，属性，索引类型 BTREE
SHOW INDEXES YIELD name, labelOrTypes, properties, type
```

### 5.3 使用约束

#### 5.3.1 唯一约束

```Cypher
CREATE CONSTRAINT constraint_example_1 FOR (movie:Movie) REQUIRE movie.title IS UNIQUE

SHOW CONSTRAINTS YIELD id, name, type, entityType, labelOrTypes, properties, ownedIndexId
```

### 5.4 创建数据库 企业版

```Cypher
CREATE database movieGraph
```
