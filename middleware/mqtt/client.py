import json

from paho.mqtt.client import Client
from loguru import logger


class MqttClient:
    def __init__(self) -> None:
        self.client = Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish

    def connect(self, host: str, port: int, user: str, passwd: str, keepalive: int = 600):
        """加载账户，连接"""

        self.client.username_pw_set(user, passwd)
        self.client.connect(host=host, port=port, keepalive=keepalive)

    def loop_start(self):
        """loop"""
        self.client.loop_start()

    def loop_forever(self):
        """循环保持"""

        self.client.loop_forever()

    def subscribe(self, topic: str):
        """订阅 topic"""

        self.client.subscribe(topic)

    def publish(self, topic, payload, qos=0):
        """发布"""

        data = json.dumps(payload, ensure_ascii=False)
        self.client.publish(topic=topic, payload=data, qos=qos)

    def add_callback(self, topic, callback):
        """向指定的 topic 添加回复/回调"""

        self.client.message_callback_add(topic, callback)

    def on_connect(self, client, userdata, flags, rc):
        """连接事件"""

        logger.info('on_connect'.center(40, '*'))
        logger.info(f'Connected with result code: {rc}')

    def on_message(self, client, userdata, msg):
        """获得消息事件，触发动作，匹配不到 message_callback_add 时使用这个"""

        logger.info('on_message'.center(40, '*'))
        payload = msg.payload.decode('utf-8')
        logger.info(f'on_message topic: {msg.topic}')
        logger.info(payload)

    def on_subscribe(self, client, userdata, mid, granted_qos):
        """订阅事件"""

        logger.info('on_subscribe'.center(40, '*'))
        logger.info('on_subscribe: qos = {granted_qos}')

    def on_unsubscribe(self, client, userdata, mid):
        """取消订阅事件"""

        logger.info('on_unsubscribe'.center(40, '*'))
        logger.info('on_unsubscribe: qos = {granted_qos}')

    def on_publish(self, client, userdata, mid):
        """发布消息事件"""

        logger.info('on_publish'.center(40, '*'))
        logger.info(f'on_publish: mid = {mid}')

    def on_disconnect(self, client, userdata, rc):
        """断开连接事件"""

        logger.info('on_disconnect'.center(40, '*'))
        logger.info('Unexpected disconnected rc = {rc}')
