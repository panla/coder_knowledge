"""
雪花算法
0 0000000-00000000-00000000-00000000-00000000-00 0000000000 000000000000
符号位 1
时间戳 41
数据中心+工作中心 10
增长序号 12

1. 第一位取0, 符号位, 0 代表非负数
2. 时间戳共41位
3. 机器位共计10位，全部表示机器时，可以表示1024台
4. twepoch 从项目开始的时间，用生成ID的时间减去开始时间作为时间戳，可以用的更久
5. -1L ^ (-1L << x) 表示 x 位的二进制数可以表示多少个数
6. 时间回拨，如果回拨较小，可以继续等待，如果较大，继续等待将不利
7. 前端精度，雪花算法结果长度可以达到19位，先转 string 再输出到前端

"""

import time
from datetime import datetime


class IdWorker:
    """
    用于生成IDs
    """

    def __init__(
            self,
            time_epoch: int,
            timestamp_shift: int,
            data_center_id_value: int,
            worker_id_value: int,
            sequence_mask: int,
            sequence: int = 0
    ):
        """初始化

        :param data_center_id: 数据中心（机器区域）ID
        :param worker_id: 机器ID
        :param sequence: 序号
        """

        self.time_epoch = time_epoch
        self.timestamp_shift = timestamp_shift
        self.sequence_mask = sequence_mask

        self.data_center_id_value = data_center_id_value
        self.worker_id_value = worker_id_value

        self._sequence = sequence

        self._last_timestamp = -1  # 上次计算的时间戳

    @staticmethod
    def _gen_timestamp() -> int:
        """生成整数时间戳(毫秒)

        :return:int timestamp
        """
        return int(time.time() * 1000)

    def _get_next_mir_time(self, last_timestamp: int) -> int:
        """等到下一毫秒
        """

        timestamp = self._gen_timestamp()
        while timestamp <= last_timestamp:
            timestamp = self._gen_timestamp()
        return timestamp

    def get_id(self) -> int:
        """获取新ID

        :return:
        """

        timestamp = self._gen_timestamp()

        # 发生了时钟回拨
        if timestamp < self._last_timestamp:
            timestamp = self._get_next_mir_time(self._last_timestamp)

        if timestamp == self._last_timestamp:
            self._sequence = (self._sequence + 1) & self.sequence_mask
            if self._sequence == 0:
                timestamp = self._get_next_mir_time(self._last_timestamp)
        else:
            self._sequence = 0

        self._last_timestamp = timestamp

        timestamp_value = (timestamp - self.time_epoch) << self.timestamp_shift

        return timestamp_value | self.data_center_id_value | self.worker_id_value | self._sequence


class IdOp:
    def __init__(
            self,
            start_time: str,
            data_center_id_bits: int = 5,
            data_center_id: int = 0,
            worker_id_bits: int = 5,
            worker_id: int = 0,
            sequence_bits: int = 12,
            sequence: int = 0
    ):
        """
        :param start_time (str): 起始时间，例如 2023-02-07 00:00:00.000000
        :param data_center_id_bits (Optional[int] = 5): 数据中心位长
        :param worker_id_bits (Optional[int] = 5): 工作ID位长
        :param sequence_bits (Optional[int] = 12): 增长序列位长
        """

        # 起始时间戳
        start_time = int(datetime.fromisoformat(start_time).timestamp() * 1000)

        # 移位偏移量
        # worker_id 偏移位数是 增长序号位数
        worker_id_shift = sequence_bits
        worker_id_value = worker_id << worker_id_shift
        # data_center_id 偏移位数是 worker_id位数 + worker_id 偏移位数
        data_center_id_shift = worker_id_shift + worker_id_bits
        data_center_id_value = data_center_id << data_center_id_shift
        # 时间戳 偏移位数是 data_center_id位数 + data_center_id 偏移位数
        timestamp_shift = data_center_id_shift + data_center_id_bits

        # 序号循环掩码, 12: 4095，同毫秒内产生的不同 ID
        sequence_mask = -1 ^ (-1 << sequence_bits)

        # 数据中心，工作ID，最大取值计算, 5: 31, 0b11111
        # sanity check
        max_data_center_id = -1 ^ (-1 << data_center_id_bits)
        max_worker_id = -1 ^ (-1 << worker_id_bits)

        if data_center_id > max_data_center_id or data_center_id < 0:
            raise ValueError(f'data_center_id {data_center_id} 越界')
        if worker_id > max_worker_id or worker_id < 0:
            raise ValueError(f'worker_id {worker_id} 越界')

        print(max_data_center_id, max_worker_id)
        self.id_worker = IdWorker(
            start_time, timestamp_shift, data_center_id_value, worker_id_value, sequence_mask, sequence
        )

    def get_id(self):

        return self.id_worker.get_id()


if __name__ == '__main__':
    # 日志打印相当的耗时

    # 测试的时候时以当前时间作为开始时间
    op = IdOp(datetime.fromtimestamp(time.time()).strftime("%Y-%m-%d %H:%M:%S.%f"))

    s_time = time.time()
    # for i in range(1, 10000000):
    for i in range(1, 10):
        id_ = op.get_id()
        # print(id_)
    print(time.time() - s_time)
    # 1000000 个耗时 0.57023 秒
    # 10000000个耗时 5.76077 秒
