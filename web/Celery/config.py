from kombu import Exchange, Queue


class MQConfig:
    USER = ''
    PASSWD = ''
    HOST = ''
    PORT = ''


class RedisConfig:
    USER = ''
    PASSWD = ''
    HOST = ''
    PORT = ''


DefaultExchangeType = 'direct'


class QueueName:
    default = 'celery-default-queue'
    test = 'celery-test-queue'
    pay = 'celery-pay-queue'
    timer = 'celery-timer-queue'


class ExchangeConst:
    default = 'celery-default-exchange'
    test = 'celery-test-exchange'
    pay = 'celery-pay-exchange'
    timer = 'celery-timer-exchange'


class RoutingKey:
    default = 'celery-default-routing'
    test = 'celery-test-routing'
    pay = 'celery-pay-routing'
    timer = 'celery-timer-routing'


DefineExchange = {
    'test': Exchange(name=ExchangeConst.test, type=DefaultExchangeType),
    'pay': Exchange(name=ExchangeConst.pay, type=DefaultExchangeType),
    'timer': Exchange(name=ExchangeConst.timer, type=DefaultExchangeType),
}


class BaseConfig:
    ###################################################################################################################
    # 1，任务队列 代理设置
    # RabbitMQ 作为任务队列
    broker_url = f'amqp://{MQConfig.USER}:{MQConfig.PASSWD}@{MQConfig.HOST}:{MQConfig.PORT}'
    # Redis 作为任务队列
    # broker_url = f'redis://{RedisConfig.USER}:{RedisConfig.PASSWD}@{RedisConfig.HOST}:{RedisConfig.PORT}/1'

    # 2，结果存储 默认，无
    # Redis 作为结果队列，Mongo
    result_backend = f'redis://{RedisConfig.USER}:{RedisConfig.PASSWD}@{RedisConfig.HOST}:{RedisConfig.PORT}/0'

    # 3，存储结果，过期时间为 一小时
    result_expires = 60 * 60

    # 4，禁用 UTC
    enable_utc = False

    # 5，时区
    timezone = 'Asia/Shanghai'

    # 6，允许的接收的内容类型/序列化程序的白名单 默认，json
    accept_content = ['json']
    # 允许结果后端的内容类型/序列化程序的白名单 默认，与 accept_content 相同
    # result_accept_content

    # 7，以秒为单位的任务硬时间限制 默认，无
    # task_time_limit = 100
    # task_soft_time_limit

    # 8，default
    # 消息没有路由或没有指定自定义队列使用的默认队列名称，默认值，celery
    task_default_queue = QueueName.default
    # 当没有为设置中键指定自定义交换时使用的交换的名称
    task_default_exchange = ExchangeConst.default
    # 当没有为设置中键指定自定义交换类型时使用的交换类型，默认值，direct
    task_default_exchange_type = DefaultExchangeType
    # 当没有为设置中键指定自定义路由键时使用的路由键
    task_default_routing_key = RoutingKey.default


class BeatConfig(BaseConfig):
    pass


class CeleryConfig(BaseConfig):

    # 9，消息路由 使用 kombu.Queue
    task_queues = (
        Queue(name=QueueName.test, exchange=DefineExchange.get('test'), routing_key=RoutingKey.test),
        Queue(name=QueueName.pay, exchange=DefineExchange.get('pay'), routing_key=RoutingKey.pay),
    )

    # 10，路由列表把任务路由到队列的路由
    task_routes = {
        'pay': {'queue': QueueName.pay, 'exchange': DefineExchange.get('pay').name, 'routing_key': RoutingKey.pay},
    }

    # 11 预取值，一次要预取的消息数乘以并发进程
    worker_prefetch_multiplier = 4

    # 12 速率 所有任务，限制每秒10次
    task_annotations = {'*': {'rate_limit': '10/s'}}
    # pay 每秒30次
    # task_annotations = {'pay': {'rate_limit': '30/s'}}
    # 自定义失败回调
    # task_annotations = {'*': {'on_failure': my_on_failure}}
    # task_annotations = (MyAnnotate(), {})

    # 13 全局默认速率
    # task_default_rate_limit = ????
