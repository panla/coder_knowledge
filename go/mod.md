# mod

[toc]

<https://www.cnblogs.com/niuben/p/12560104.html>

## command

```bash
# 初始化go.mod
go mod init

# 更新依赖文件
go mod tidy

# 下载依赖文件
go mod download

# 将依赖转移至本地的vendor文件
go mod vendor

# 手动修改依赖文件
go mod edit

# 打印依赖图
go mod graph

# 校验依赖
go mod verify

## 依赖更新
go mod tidy
go mod download
go mod vendor

```
