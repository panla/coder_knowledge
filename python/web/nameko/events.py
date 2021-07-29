"""
event_handler 方式
不阻塞，但直接返回了结果
不能收到 event 的返回值
适合不管处理结果的场景
不过可以做回调，来起到返回处理结果的作用
"""

import time

from nameko.rpc import rpc
from nameko.events import EventDispatcher, event_handler

from .config import ENV


class EventService:
    name = ENV.EventService.NAME
    service_event_type = ENV.EventService.EVENT_TYPE
    event_dispatch = EventDispatcher()

    @rpc
    def execute(self, *args, **kwargs):
        self.event_dispatch(self.service_event_type, *args, **kwargs)
        return True

    @event_handler(source_service=ENV.EventService.NAME, event_type=ENV.EventService.EVENT_TYPE)
    def handler(self, *args, **kwargs):
        for i in range(1, 6):
            print(time.time(), i)
            print(args)
            print(kwargs)
            time.sleep(1)
        # 这里的返回值无效
        # return True
