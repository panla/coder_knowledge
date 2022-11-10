# command

[toc]

## 1 go

<https://golang.org/doc.cmd>

### 1.1 编译包和依赖

```bash
go build
```

### 1.2 编译并运行程序

```bash
go run
```

### 1.3 移除对象文件

```bash
go clean
```

### 1.4 显示包或者符号的文档

```bash
go doc
```

### 1.5 打印go的环境信息

```bash
go env
```

### 1.6 显示 go 版本

```bash
go version
```

### 1.7 下载并安装包和依赖

```bash
go get
```

### 1.8 编译并安装包和依赖

```bash
go install
```

### 1.9 运行 gofmt 进行格式化

```bash
go fmt
```

### 1.10 运行测试

```bash
go test
```

### 1.11 列出包

```bash
go list
```

### 1.12 go 提供的工具

```bash
go tool
```

### 1.13 运行 go tool fix

```bash
go fix
```

### 1.14 启动错误报告

```bash
go bug
```

### 1.15 从 processing source 生成 go 文件

```bash
go generate
```

## 2 go mod

### 2.1 初始化go.mod

```bash
# initialize new module in current directory
go mod init github.com/zeromicro/go-zero
```

### 2.2 更新依赖文件

```bash
# add missing and remove unused modules
go mod tidy
```

### 2.3 下载依赖文件

```bash
# download modules to local cache
go mod download
```

### 2.4 将依赖转移至本地的 vendor 文件

```bash
# make vendored copy of dependencies
go mod vendor
```

### 2.5 手动修改依赖文件

```bash
# edit go.mod from tools or scripts
go mod edit
```

### 2.6 打印依赖图

```bash
# print module requirement graph
go mod graph
```

### 2.7 校验依赖

```bash
# verify dependencies have expected content
go mod verify
```

### 2.8 为何需要此依赖

```bash
# explain why packages or modules are needed
go mod why
```

### 2.9 依赖更新

```bash
go mod tidy
go mod download
go mod vendor
```
