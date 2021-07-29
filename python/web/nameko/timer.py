from nameko.timer import timer

from .config import ENV


class TimerService:
    name = ENV.TimerService.NAME

    #普通定时器
    @timer(10)
    def execute(self):
        print('I am a common timer')
