# 声明模型

## doc

<https://gorm.io/zh_CN/docs/models.html>

## 字段标签

| tagName | desc | example |
| :-: | :-: | :-: |
| column | 指定db列名 |
| type | 列数据类型 | bool,int,uint |
| serializer | 指定将数据序列化或反序列化到数据库中的序列化器 | serializer:json/gorb/unixtime |
| size | 定义列数据类型的大小或长度 | size:256 |
| primaryKey | 将列定义为主键 |  |
| unique | 将列定义为唯一键 |  |
| uniqueIndex | 定义列为唯一索引 |  |
| default | 定义列的默认值 |  |
| precision | 指定列的精度 |  |
| scale | 指定列的大小 |  |
| not null | 指定列非空 |  |
| autoIncrement | 指定自增长 |  |
| autoIncrementIncrement | 指定自增长步长 |  |
| embedded | 嵌套 |  |
| embeddedPrefix | 嵌套前缀 | embeddedPrefix:author_ |
| autoCreateTime | 自动设置创建时间，对于 int 字段会追踪时间戳秒数，nano/milli 纳秒，毫秒 | autoCreateTime:milli |
| index | 创建索引 | 见索引 |
| check | 检查约束 | check:age > 13，见约束 |
| <- | 写入权限 | <-:create 只创建，<-:update 只更新，<-:false，无写入权限，<- 创建和更新权限 |
| -> | 读取权限 | ->:false 无权限 |
| - | 忽略该字段 | - 无读写，-:migration，无迁移权限，-:all 无读写迁移权限 |
| comment | 迁移时的字段注释 |  |

## index

```text
`gorm:"index:idx_name,unique"`

```
