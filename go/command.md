# command

## go

<https://golang.org/doc.cmd>

```bash
go build
    编译包和依赖
go run
    编译并运行程序
go clean
    移除对象文件
go doc
    显示包或者符号的文档
go env
    打印go的环境信息
go version
    显示 go 版本
go get
    下载并安装包和依赖
go install
    编译并安装包和依赖
go fmt
    运行 gofmt 进行格式化
go test
    运行测试
go list
    列出包
go tool
    go 提供的工具
go fix
    运行 go tool fix
go bug
    启动错误报告
go generate
    从 processing source 生成 go 文件
```

## go mod

```bash
# 初始化go.mod
# initialize new module in current directory
go mod init github.com/zeromicro/go-zero

# 更新依赖文件
# add missing and remove unused modules
go mod tidy

# 下载依赖文件
# download modules to local cache
go mod download

# 将依赖转移至本地的vendor文件
# make vendored copy of dependencies
go mod vendor

# 手动修改依赖文件
# edit go.mod from tools or scripts
go mod edit

# 打印依赖图
# print module requirement graph
go mod graph

# 校验依赖
# verify dependencies have expected content
go mod verify

# 为何需要此依赖
# explain why packages or modules are needed
go mod why

## 依赖更新
go mod tidy
go mod download
go mod vendor
```
