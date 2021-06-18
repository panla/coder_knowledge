# pika

## task example

1 **package.py**

```python
import pika
from pika import credentials


connection_params = {
    'host': '127.0.0.1',
    'port': 15672,
    'credentials': credentials.PlainCredentials(username='root', password='root')
}

connection = pika.BlockingConnection(pika.ConnectionParameters(**connection_params))

channel = connection.channel()
channel.basic_qos(prefetch_count=1)

properties = pika.BasicProperties(delivery_mode=2)

```

2 **task.py**

```python
import sys

from package import channel, connection, properties


message = ' '.join(sys.argv[1:]) or 'Hello World'

channel.queue_declare(queue='hello', durable=True)
channel.basic_publish(exchange='', routing_key='hello', body=message, properties=properties)
print(f'[X] Send {message}')

connection.close()

```

3 **worker.py**

```python
import time

from package import channel


channel.queue_declare(queue='hello', durable=True)

def callback(ch, method, properties, body):
    print("[x] Received %r" % body)
    time.sleep(len(body))
    print('[X] Done')


channel.basic_consume(queue='hello', on_message_callback=callback, auto_ack=True)
print('[*] Waiting for messages. To exit press CTRL+C')
channel.start_consuming()

```
