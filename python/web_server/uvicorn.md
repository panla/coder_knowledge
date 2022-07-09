# uvicorn

## 配置参数

```text
--host --port --reload --workers
--loop --http
--limit-concurrency  在发出 HTTP 503 响应之前允许的最大并发连接或任务数。即使在资源过剩的情况下，也可用于确保已知的内存使用模式
--limit-max-requests 终止进程前的最大服务请求数。
--backlog            积压的最大连接数。与大量传入流量相关。默认值： 2048
--timeout-keep-alive 如果在此超时时间内没有收到新数据，则关闭 Keep-Alive 连接。默认值： 5。
```

## 运行服务

```python
import json

async def app(scope, receive, send):
    assert scope['type'] == 'http'

    await send({
        'type': 'http.response.start',
        'status': 200,
        'headers': [
            [b'content-type', b'application/json'],
        ],
    })

    await send({
        'type': 'http.response.body',
        'body': bytes(json.dumps({'id': 1}), encoding='utf-8')
    })
```

```bash
uvicorn server:app
```
