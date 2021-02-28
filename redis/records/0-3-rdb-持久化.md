# Redis 持久化 RDB

## RDB

在指定时间间隔内把内存中的数据快照写到磁盘 snapshot 快照，恢复时把快照文件读进内存
Redis会fork一个子进程来持久化，会先把数据写入到一个临时文件中，待持久化过程结束，把临时文件替换上次持久化好的文件
优点：
1 适合大规模数据恢复
2 对数据完整性要求不高
缺点：
1 最后一次持久化后的数据可能会丢失
2 fork 进程时占用资源

## 配置

- save 900 1      900 秒内至少1个key进行过修改，就进行持久化操作
- save 300 10     300 秒内至少10个key进行过修改，就进行持久化操作
- save 60 100000  60  秒内至少10000个key进行过修改，就进行持久化操作
- `stop-writes-on-bgsave-error` yes 持久化出错继续工作
- rdbcompression yes 是否压缩rdb文件
- rdbnchecksum yes 保存rdb文件出错时进行错误检查校验
- dir ./ rdb 文件保存目录

## 产生

- save
- flushall
- 退出

## 恢复

把 rdb 文件放在redis启动目录
