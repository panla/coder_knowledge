# MySQL 工具的命令

[TOC]

## 导入导出

导出

```bash
mysqldump -u root -p db_name --set-gtid-purged=OFF --column-statistics=0 > db_name.sql
```

只导出结构

```bash
mysqldump -u root -p -d db_name --set-gtid-purged=OFF --column-statistics=0 > db_name.sql
```

```text
--set-gtid-purged=OFF --column-statistics=0 部分时候需要
```

导入

```bash
mysqld -u root -p db_name < db_name.sql
```
