# Cypher

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

CREATE(movie:Movie {title: "The Matrix", released: 1997})
RETURN movie
```

### 2.2 匹配 MATCH

```Cypher
# 匹配查询 title=The Matrix 的电影

MATCH (movie:Movie {title: "The Matrix"})
RETURN movie
```
