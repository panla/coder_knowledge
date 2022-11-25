# install

[toc]

## download

<https://studygolang.com/dl>

## env

```text
export GOROOT=/opt/go
export PATH=$PATH:$GOROOT/bin
export GOPATH=/home/user/study/go_path
# 配置 GOPROXY 环境变量
# export GOPROXY=https://goproxy.io,direct
export GOPROXY=https://goproxy.cn,direct
# GOPROXY=https://mirrors.aliyun.com/goproxy/
export GO111MODULE=on
```

## hello world

```bash
cd ~/study/go_project
mkdir tour && cd tour
go mod init example.org/tour

touch main.go
```

```go
package main

import (
    "fmt"
)

func main() {
    fmt.Println("Hello World")
}
```

```bash
go install example.org/tour

cd ~/study/go_path/bin

./tour
```
