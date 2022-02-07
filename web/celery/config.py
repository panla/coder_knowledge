from kombu import Exchange, Queue
from celery.schedules import crontab


class MQConfig:
    pass


class RedisConfig:
    pass


class CeleryConfig:
    # 任务队列
    broker_url = f'amqp://{MQConfig.USER}:{MQConfig.PASSWD}@{MQConfig.HOST}:{MQConfig.PORT}'
    # 结果存储
    result_backend  = f'redis://{RedisConfig.USER}:{RedisConfig.PASSWD}@{RedisConfig.HOST}:{RedisConfig.PORT}/0'
    # 存储结果，过期时间为 一小时
    result_expires = 60 * 60
    # 禁用 UTC
    enable_utc = False
    # 时区
    timezone = 'Asia/Shanghai'
    # 接收 JSON
    accept_content = ['json']

    DefaultExchangeType = 'direct'

    class ExchangeConst:
        default = 'celery-default'
        pay = 'celery-pay'

    class RoutingKeyConst:
        default = 'default-tasks'
        pay = 'pay-tasks'

    class QueueNameConst:
        default = 'celery-default-tasks'
        pay = 'celery-pay-tasks'


    define_exchange = {
        'default': Exchange(ExchangeConst.default, type=DefaultExchangeType),
        'pay': Exchange(ExchangeConst.pay, type=DefaultExchangeType)
    }

    task_queues = (
        Queue(QueueNameConst.default, routing_key=RoutingKeyConst.default, exchange=define_exchange.get('default')),
        Queue(QueueNameConst.pay, routing_key=RoutingKeyConst.pay, exchange=define_exchange.get('pay'))
    )

    task_routes = {
        'start_task': {'exchange': define_exchange.get('pay').name, 'routing_key': RoutingKeyConst.pay}
    }

    beat_schedule = {
        'a': {
            'task': 'services.tasks.hello',
            'schedule': crontab(minute='*/10') # 每 10 分钟执行一次
        },
        'b': {
            'task': 'services.tasks.hello',
            'schedule': crontab(minute=10) # 每个小时的 第 10 分钟执行一次
        }
    }
