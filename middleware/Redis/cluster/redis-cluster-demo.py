import time
import asyncio

# from redis.cluster import RedisCluster, ClusterNode
from redis.asyncio.cluster import RedisCluster, ClusterNode


class RedisConfig:

    MAX_CONNECTIONS = 100000
    PASSWORD = '2YiptpmJDevSSSRSRNBlKA'
    HOST = '192.168.9.99'
    PORT = 9202
    PORTS = [i for i in range(9201, 9206)]


REDIS_CONNECTION_PARAMS = {
    'max_connections': RedisConfig.MAX_CONNECTIONS,
    'password': RedisConfig.PASSWORD,
    'host': RedisConfig.HOST,
    'port': RedisConfig.PORT,
    'encoding': 'utf-8',
    'decode_responses': True
}


class ConnectionCache:

    _instance = None

    def __new__(cls) -> RedisCluster:

        if not cls._instance:
            nodes = list()
            for port in RedisConfig.PORTS:
                nodes.append(ClusterNode(host=RedisConfig.HOST, port=port))

            cls._instance = RedisCluster(startup_nodes=nodes, read_from_replicas=True, **REDIS_CONNECTION_PARAMS)

        return cls._instance


class BaseRedis:

    PREFIX_KEY = 'base'

    def __init__(self) -> None:
        self._name = None
        self.client = ConnectionCache()

    @property
    def name(self):
        if not self._name:
            raise Exception('no name')
        return self._name

    @name.setter
    def name(self, value):
        self._name = f'{self.PREFIX_KEY}:{value}'

    def get(self):
        return self.client.get(name=self.name)

    def set(self, value):
        return self.client.set(name=self.name, value=value)

    def set_nx(self, value):
        return self.client.setnx(name=self.name, value=value)


class CarRedis(BaseRedis):

    PREFIX_KEY = 'cars'


async def async_main():

    car = CarRedis()

    for i in range(1000):
        car.name = i
        await car.set(time.time())

        print(await car.get())


def sync_main():
    car = CarRedis()

    for i in range(1000):
        car.name = i
        car.set(time.time())

        print(car.get())


s_time = time.time()
asyncio.run(async_main())
# sync_main()
e_time = time.time()

print(e_time - s_time)
