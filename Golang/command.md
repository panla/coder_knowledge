# command

[toc]

## 1 go

<https://golang.org/doc.cmd>

- go build 编译包和依赖
- go run 编译并运行程序
- go clean 移除对象文件
- go doc 显示包或者符号的文档
- go env 打印go的环境信息
- go version 显示 go 版本
- go get 下载并安装包和依赖
- go install 编译并安装包和依赖
- go fmt 运行 gofmt 进行格式化
- go test 测试
- go list 列出包
- go tool go 提供的工具
- go fix 运行 go tool fix
- go bug 启动错误报告
- go generate 从 processing source 生成 go 文件

## 2 go mod

- go mod init 初始化go.mod
  - ```go mod init github.com/zeromicro/go-zero```
- go mod tidy 更新依赖文件
- go mod download 下载依赖文件
- go mod vendor 将依赖转移至本地的 vendor 文件
- go mod edit 手动修改依赖文件
- go mod graph 打印依赖图
- go mod verify 校验依赖
- go mod why 为何需要此依赖

### 2.1 依赖更新

```bash
go mod tidy
go mod download
go mod vendor
```
