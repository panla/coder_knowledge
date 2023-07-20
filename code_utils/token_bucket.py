"""
令牌桶
"""

import time

class TokenBucket:

    def __init__(self, rate: int, capacity: int) -> None:
        self.rate = rate = rate
        self.capacity = capacity

        self.current_amount = 0
        self.last_consume_time = time.time()

    def consume(self, amount: int) -> bool:

        num = int((time.time() - self.last_consume_time)) * self.rate
        self.current_amount = min(num + self.current_amount, self.capacity)

        if amount > self.current_amount:
            # 没有足够令牌
            return False

        self.last_consume_time = time.time()
        self.current_amount -= amount
        return True
