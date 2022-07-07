import time
from datetime import datetime

from loguru import logger

# 64位ID的划分
WORKER_ID_BITS = 5
DATA_CENTER_ID_BITS = 5
SEQUENCE_BITS = 12

# 最大取值计算, 31, 2**5-1, 0b11111
MAX_WORKER_ID = -1 ^ (-1 << WORKER_ID_BITS)
MAX_DATA_CENTER_ID = -1 ^ (-1 << DATA_CENTER_ID_BITS)

# 移位偏移计算
WORKER_ID_SHIFT = SEQUENCE_BITS
DATA_CENTER_ID_SHIFT = SEQUENCE_BITS + WORKER_ID_BITS
TIMESTAMP_LEFT_SHIFT = SEQUENCE_BITS + WORKER_ID_BITS + DATA_CENTER_ID_BITS

# 序号循环掩码
SEQUENCE_MASK = -1 ^ (-1 << SEQUENCE_BITS)

# 起始时间戳 毫秒
TIME_EPOCH = int(datetime.fromisoformat("2022-07-07 00:00:00.000000").timestamp() * 1000)
# TIME_EPOCH = int(datetime.fromisoformat("2022-07-07 00:00:00.000000").timestamp())


class InvalidSystemClock(Exception):
    pass


class IdWorker(object):
    """
    用于生成IDs
    """

    def __init__(self, data_center_id, worker_id, sequence: int = 0):
        """初始化

        :param data_center_id: 数据中心（机器区域）ID
        :param worker_id: 机器ID
        :param sequence: 序号
        """

        # sanity check
        if worker_id > MAX_WORKER_ID or worker_id < 0:
            raise ValueError(f'worker_id {worker_id} 越界')

        if data_center_id > MAX_DATA_CENTER_ID or data_center_id < 0:
            raise ValueError(f'data_center_id {data_center_id} 越界')

        self.data_center_id = data_center_id
        self.worker_id = worker_id
        self.sequence = sequence

        self.last_timestamp = -1  # 上次计算的时间戳

    @staticmethod
    def _gen_timestamp() -> int:
        """生成整数时间戳(毫秒)

        :return:int timestamp
        """
        return int(time.time() * 1000)
        # return int(time.time())

    def _til_next_mir_time(self, last_timestamp) -> int:
        """等到下一毫秒
        """

        timestamp = self._gen_timestamp()
        while timestamp <= last_timestamp:
            timestamp = self._gen_timestamp()
        return timestamp

    def get_id(self) -> str:
        """获取新ID

        :return:
        """

        timestamp = self._gen_timestamp()

        # 时钟回拨
        if timestamp < self.last_timestamp:
            logger.error('clock is moving backwards. Rejecting requests until {}'.format(self.last_timestamp))
            raise InvalidSystemClock

        if timestamp == self.last_timestamp:
            self.sequence = (self.sequence + 1) & SEQUENCE_MASK
            if self.sequence == 0:
                timestamp = self._til_next_mir_time(self.last_timestamp)
        else:
            self.sequence = 0

        self.last_timestamp = timestamp

        a = (timestamp - TIME_EPOCH) << TIMESTAMP_LEFT_SHIFT
        b = self.data_center_id << DATA_CENTER_ID_SHIFT
        c = self.worker_id << WORKER_ID_SHIFT

        return a | b | c | self.sequence


if __name__ == '__main__':
    worker = IdWorker(1, 2, 0)

    for i in range(1, 100):
        print(worker.get_id())
