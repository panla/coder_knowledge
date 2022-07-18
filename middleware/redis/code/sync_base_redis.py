from datetime import timedelta
from typing import Union, Optional, Dict

from redis.client import Redis
from redis.connection import ConnectionPool


class RedisConfig:
    MAX_CONNECTIONS = 100000
    USER = None
    PASSWD = '12345678'
    HOST = 'ip'
    PORT = 6379


REDIS_CONNECTION_PARAMS = {
    'max_connections': RedisConfig.MAX_CONNECTIONS,
    'username': RedisConfig.USER,
    'password': RedisConfig.PASSWD,
    'host': RedisConfig.HOST,
    'port': RedisConfig.PORT,
    'encoding': 'utf-8',
    'decode_responses': True
}


class Pool:
    cache: Dict[str, ConnectionPool] = dict()
    instance = None

    def __init__(self, db: str = '0') -> None:
        self.db = db

    def __new__(cls, db: str = '0'):

        if not cls.instance:
            cls.instance = super().__new__(cls)

        if not cls.cache.get(db):
            cls.instance.cache[db] = ConnectionPool(db=db, **REDIS_CONNECTION_PARAMS)

        return cls.instance

    def pool(self):
        return self.cache.get(str(self.db))


class BaseRedis(object):
    DB = 0
    PREFIX_KEY = ''

    def __init__(self) -> None:
        self._name = None

        self.client: Redis = Redis(connection_pool=Pool(str(self.DB)).pool())

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = f'{self.PREFIX_KEY}:{value}'

    def expire(self, seconds):
        """
        Set an expired flag on key ``name`` for ``time`` seconds. ``time``
        can be represented by an integer or a Python timedelta object.
        """

        return self.client.expire(name=self.name, time=seconds)

    def delete(self):
        """Delete one or more keys specified by ``names``"""

        return self.client.delete(self.name)

    def exists(self):
        """Returns the number of ``names`` that exist"""

        return self.client.exists(self.name)

    def ttl(self):
        """Returns the number of seconds until the key ``name`` will expire"""

        return self.client.ttl(name=self.name)

    def get(self):
        """
        Return the value at key ``name``, or None if the key doesn't exist
        """

        return self.client.get(name=self.name)

    def set(
            self,
            value,
            ex: Union[int, timedelta] = None,
            px: Union[int, timedelta] = None,
            nx: bool = False,
            xx: bool = False,
            get: bool = False
    ):
        """Set the value at key ``name`` to ``value``

        ``ex`` sets an expired flag on key ``name`` for ``ex`` seconds.

        ``px`` sets an expired flag on key ``name`` for ``px`` milliseconds.

        ``nx`` if set to True, set the value at key ``name`` to ``value`` only
            if it does not exist.

        ``xx`` if set to True, set the value at key ``name`` to ``value`` only
            if it already exists.

        ``get`` if True, set the value at key ``name`` to ``value`` and return
            the old value stored at key, or None if the key did not exist.
            (Available since Redis 6.2)
        """

        return self.client.set(name=self.name, value=value, ex=ex, px=px, nx=nx, xx=xx, get=get)

    def set_nx(self, value):
        """Set the value of key ``name`` to ``value`` if key doesn't exist"""

        return self.client.setnx(name=self.name, value=value)

    def incr_by(self, amount: int = 1):
        """value += amount"""

        return self.client.incrby(name=self.name, amount=amount)

    def hash_set(self, key: Optional[str] = None, value: Optional[str] = None, mapping: Optional[dict] = None):
        """
        Set ``key`` to ``value`` within hash ``name``,
        ``mapping`` accepts a dict of key/value pairs that that will be
        added to hash ``name``.
        Returns the number of fields that were added.
        """

        return self.client.hset(name=self.name, key=key, value=value, mapping=mapping)

    def hash_get(self, key):
        """hash, Return the value of ``key`` within the hash ``name``"""

        return self.client.hget(name=self.name, key=key)

    def hash_get_all_values(self):
        """hash, Return a Python dict of the hash's name/value pairs"""

        return self.client.hgetall(name=self.name)

    def hash_del_key(self, keys: list):
        """hash, Delete ``keys`` from hash ``name``

        have * need list         ([key, key, key])
        no   * need multiple key (key, key, key) ot *[key, key, key]
        """

        self.client.hdel(self.name, *keys)

    def list_left_range(self, start: int = 0, end: int = -1):
        """list, Return a slice of the list ``name`` between position ``start`` and ``end``"""

        return self.client.lrange(name=self.name, start=start, end=end)

    def list_push(self, values: list, is_right: bool = True):
        """list, Push ``values`` onto the tail/head of the list ``name``"""

        if is_right:
            return self.client.rpush(self.name, *values)
        else:
            return self.client.lpush(self.name, *values)

    def list_set(self, index, value):
        """list, Set element at ``index`` of list ``name`` to ``value``"""

        return self.client.lset(name=self.name, index=index, value=value)

    def set_bit(self, offset: int, value: Union[int, bool]):
        """Flag the ``offset`` in ``name`` as ``value``.

        Returns an integer indicating the previous value of ``offset``.

        value = value and 1 or 0.
        value in [0, 1]
        """

        return self.client.setbit(name=self.name, offset=offset, value=value)

    def count_bit(self, start: int = 0, end: int = -1):
        """Returns the count of set bits in the value of ``key``.

        Optional ``start`` and ``end`` parameters indicate which bytes to consider

        mode in [BYTE, BIT]. Starting with Redis version 7.0.0: Added the BYTE|BIT option.
        """

        return self.client.bitcount(key=self.name, start=start, end=end)
