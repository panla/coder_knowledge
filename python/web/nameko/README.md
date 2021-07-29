# nameko

Nameko 是一个用 Python 构建微服务的框架。

它内置支持：

- AMQP 上的 RPC
- AMQP 上的异步事件（发布-订阅）
- 简单的 HTTP GET 和 POST

## 服务

- [rpc](./rpc.py)
- [events](./events.py)
- [http](./http.py)
- [timer](./timer.py)

## 运行

```bash
sh ./run.sh

nameko run server --config config.yaml
``
