"""
RpcProxy

服务间调用

会耗时，并阻塞，但可以收到另一个服务的返回结果
"""

import time

from nameko.rpc import rpc, RpcProxy

from .config import ENV


class RpcXService:
    name = ENV.RpcXService.NAME

    @rpc
    def execute(self, value):
        for i in range(5):
            time.sleep(1)
            print(f'{i} rpc_x_service received {value}')
        return f'rpc_x_service received {value}'


class RpcYService:
    name = ENV.RpcYService.NAME

    x = RpcProxy(ENV.RpcXService)

    @rpc
    def execute(self, value):
        print(f'rpc_y_service received {value}')
        return self.x.execute(value)
