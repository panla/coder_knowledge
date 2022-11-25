# nGQL 概述

[toc]

## 1 图模式

### 1.1 单点模式

```nGQL
(point)
```

### 1.2 多点关联模式，点与点之间关系

```nGQL
(point_a)-[]->(point_b)

(point_a)-[]->(point_b)<-[]-(point_c)

(a)-[]->()<-[]-(c)
```

### 1.3 Tag 标签模式

```nGQL
(point_a:point)-[]->(point_b:point)

(user:Python:Java)-[]->(point_b:point)
```

### 1.4 边模式，关系

```nGQL
(a)-[]-(b)

(a)-[r]->(b)

(a)-[r:relation]->(b)

# 可选的多种边
(a)-[r:TYPE1|TYPE2]->(b)
```

### 1.5 变长模式

```nGQL
(a)-[*2]->(b)
=== 三点两边
(a)-[]->()-[]->(b)

(a)-[*3..5]->(b)
===四点三边，或五点四边，或六点五边

===两点一边，或三点两边，或四点三边，或五点四边，或六点五边
(a)-[*..5]->(b)
```

### 1.6 Path, 路径变量

一系列连接的点和边称为路径

```nGQL
p = (a)-[*3..5]->(b)
```

## 2 Style

```text
1   使用单数名词命名 Tag，用原型动词或动词短语构成 Edge type
    p=(v:player)-[e:follow]-(v2)

2   标识符用蛇形命名法，以下划线（_）连接单词，且所有字母小写
    MATCH (v:basketball_team)

3   语法关键词大写，变量小写
    MATCH (v:player) RETURN LIMIT 5;
```

```text
1   标识符区分大小写，关键字与函数不区分大小写
    MATCH (v:player{name: "Tim Duncan", age: 42})-[e:follow]-> \
    ()-[e2:serve]->()<--(v2) \
    RETURN v, e, v2;

2   `\` 换行

3   分行写 Pattern 时，在表示边的箭头右侧换行，而不是左侧
    MATCH (v)<-[:follow]-() RETURN v;

4   将无需查询的点和边匿名化
    MATCH (v:player)-[:follow]->() RETURN v;
```

```text
1   字符串用双引号包裹
    RETURN "Hello Nebula"

2   用 ; 结尾

3   有 | 管道符时，只在最后一行用 ; 结尾

4   在包含自定义变量的复合语句中，用英文分号结束定义变量的语句。需要分号不能使用管道符
```
