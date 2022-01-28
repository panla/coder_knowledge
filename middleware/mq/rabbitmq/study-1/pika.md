# pika

[toc]

## task example

1 **package.py**

```python
import pika

from pika import credentials, BlockingConnection, BasicProperties
from pika.adapters.blocking_connection import BlockingChannel


class ProducerConsumerTool(object):

    CONNECTION_PARAMS = {
        'host': '127.0.0.1',
        'port': 15672,
        'credentials': credentials.PlainCredentials(username='root', password='root')
    }

    QUEUE = 'hello'
    ROUTING_KEY = 'hello'
    PREFETCH_SIZE = 1
    PREFETCH_COUNT = 1
    # 忙时不分配任务

    DELIVERY_MODE = 2
    # delivery_mode=2 消息持久化

    def __init__(self) -> None:
        self.connection: BlockingConnection = BlockingConnection(pika.ConnectionParameters(**self.CONNECTION_PARAMS))
        self.channel: BlockingChannel = self.get_channel
        self.properties: BasicProperties = BasicProperties(delivery_mode=self.DELIVERY_MODE)

    @property
    def get_channel(self) -> BlockingChannel:
        _channel = self.connection.channel()
        _channel.basic_qos(prefetch_count=self.PREFETCH_COUNT)

        _channel.queue_declare(queue=self.QUEUE, durable=True)
        return _channel

    def produce_msg(self, message):
        self.channel.basic_publish(exchange='', routing_key=self.ROUTING_KEY, body=message, properties=self.properties)
        self.connection.close()

    def produce_msgs(self, messages):
        for message in messages:
            self.channel.basic_publish(exchange='', routing_key=self.ROUTING_KEY, body=message, properties=self.properties)
        self.connection.close()

    def consume(self, callback):
        print('[*] Waiting for messages. To exit press CTRL+C')
        self.channel.basic_consume(queue=self.QUEUE, on_message_callback=callback, auto_ack=True)
        self.channel.start_consuming()


class PublishSubscribeTool(object):
    CONNECTION_PARAMS = {
        'host': '127.0.0.1',
        'port': 15672,
        'credentials': credentials.PlainCredentials(username='root', password='root')
    }

    EXCHANGE = 'logs'
    EXCHANGE_TYPE = 'fanout'

    def __init__(self) -> None:
        self.connection: BlockingConnection = BlockingConnection(pika.ConnectionParameters(**self.CONNECTION_PARAMS))
        self.channel: BlockingChannel = self.get_channel

    @property
    def get_channel(self) -> BlockingChannel:
        _channel = self.connection.channel()
        _channel.exchange_declare(exchange=self.EXCHANGE, exchange_type=self.EXCHANGE_TYPE)

        result = _channel.queue_declare(queue='', durable=True, exclusive=True)
        self.queue_name = result.method.queue
        _channel.queue_bind(queue=self.queue_name, exchange=self.EXCHANGE)
        return _channel

    def publish_msg(self, message):
        self.channel.basic_publish(exchange=self.EXCHANGE, routing_key='', body=message)
        self.connection.close()

    def publish_msgs(self, messages):
        for message in messages:
            self.channel.basic_publish(exchange=self.EXCHANGE, routing_key='', body=message)
        self.connection.close()

    def receive(self, callback):
        print('[*] Waiting for messages. To exit press CTRL+C')
        self.channel.basic_consume(queue=self.queue_name, on_message_callback=callback, auto_ack=True)
        self.channel.start_consuming()

```

2 **producer.py**

```python
from package import ProducerConsumerTool

mq = ProducerConsumerTool()

# mq.produce_msg('123')
mq.produce_msgs(['123', '456'])

```

3 **consumer.py**

```python
import time
import sys

from package import ProducerConsumerTool


def callback_func(ch, method, properties, body):
    print("[x] Received %r" % body)

    for i in body.decode():
        print(i)
        time.sleep(0.2)

    print('[X] Done')
    # ch.basic_ack(delivery_tag=method.delivery_tag)
    # auto_ack=True 时关闭手动消息确认


if __name__ == '__main__':
    try:
        mq = ProducerConsumerTool()

        mq.consume(callback=callback_func)

    except KeyboardInterrupt:
        sys.exit('1')

```

4 **receiver.py**

```python
import time
import sys

from package import PublishSubscribeTool


def callback_func(ch, method, properties, body):
    print("[x] Received %r" % body)

    for i in body.decode():
        print(i)
        time.sleep(0.2)

    print('[X] Done')


if __name__ == '__main__':
    try:
        mq = PublishSubscribeTool()

        mq.receive(callback=callback_func)

    except KeyboardInterrupt:
        sys.exit('1')
```

5 **publish.py**

```python
from package import PublishSubscribeTool


client = PublishSubscribeTool()

client.publish_msgs(['789', '987'])
```
