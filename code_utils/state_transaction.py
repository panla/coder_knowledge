"""
https://blog.csdn.net/StoryMonster/article/details/99443480
"""

class FsmState:

    def enter(self, event, fsm):
        pass

    def exit(self, fsm):
        pass


class DvdPowerOn(FsmState):

    def enter(self, event, fsm):
        print('power on and dvd is power on')


class DvdPlaying(FsmState):

    def enter(self, event, fsm):
        print('play and dvd is goding to play')

    def exit(self, fsm):
        print('stop play and dvd stop play')


class DvdPausing(FsmState):

    def enter(self, event, fsm):
        print('pause and dvd is going to pause')

    def exit(self, fsm):
        print('stop pause and dvd is stopped pause')


class DvdPowerOff(FsmState):

    def enter(self, event, fsm):
        print('power off and dvd is power off')


class FsmFinalState(FsmState):

    def enter(self, event, fsm):
        print('FsmFinalState')


class FsmEvent:
    pass


class PowerOnEvent(FsmEvent):

    pass


class PowerOffEvent(FsmEvent):

    pass


class PlayEvent(FsmEvent):

    pass


class PauseEvent(FsmEvent):

    pass


from collections import namedtuple

Transaction = namedtuple(
    'Transaction', ['pre_state', 'event', 'next_state']
)

class FsmExecption(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class FSM:

    GLOBAL_TRANSACTION_TABLE = list()
    STATE_TRANSACTION_TABLE = list()

    def __init__(self, context) -> None:
        self.context = context

        self.current_state = None
        self.working_state = FsmState

    def add_gloabl_transaction(self, event, end_state):
        """全局转换，直接进入到结束状态"""

        if not issubclass(end_state, FsmFinalState):
            raise FsmExecption('The state should be FsmFinalState')
        transaction = Transaction(pre_state=self.working_state, event=event, next_state=end_state)
        self.GLOBAL_TRANSACTION_TABLE.append(transaction)

    def add_transaction(self, pre_state, event, next_state):
        """添加 转移规则，从前 pre_state 经过 event 转变为 next_state

        :param pre_state  : FsmState 的子类，前一个状态
        :param event      : FsmEvent 的子类，事件
        :param next_state : FsmState 的子类，下一个状态
        """

        if issubclass(pre_state, FsmFinalState):
            raise FsmExecption('It`s not allowed to add transaction after Final State Node')
        self.STATE_TRANSACTION_TABLE.append(Transaction(pre_state=pre_state, event=event, next_state=next_state))

    def process_event(self, event: FsmEvent):
        """执行事件

        Args:
            event (FsmEvent): 事件

        Raises:
            FsmExecption: 异常
        """

        for transaction in self.GLOBAL_TRANSACTION_TABLE:
            if isinstance(event, transaction.event):
                self.current_state = transaction.next_state()
                self.current_state.enter(event, self)
                self.clear_transaction_table()
                return
        for transaction in self.STATE_TRANSACTION_TABLE:
            if isinstance(self.current_state, transaction.pre_state) and isinstance(event, transaction.event):
                self.current_state.exit(self.context)
                self.current_state = transaction.next_state()
                self.current_state.enter(event, self)
                if isinstance(self.current_state, FsmFinalState):
                    self.clear_transaction_table()
                return
        raise FsmExecption('Transaction not found')

    def clear_transaction_table(self):
        """清除当前转换表"""

        print('清除转换表')
        self.GLOBAL_TRANSACTION_TABLE = list()
        self.STATE_TRANSACTION_TABLE = list()
        self.current_state = None

    def run(self):
        if len(self.STATE_TRANSACTION_TABLE) == 0:
            return
        self.current_state = self.STATE_TRANSACTION_TABLE[0].pre_state()
        self.current_state.enter(None, self)

    def is_running(self):
        return self.current_state is not None

    def next_state(self, event: FsmEvent):
        for transaction in self.GLOBAL_TRANSACTION_TABLE:
            if isinstance(event, transaction.event):
                return transaction.next_state
        for transaction in self.STATE_TRANSACTION_TABLE:
            if isinstance(self.current_state, transaction.pre_state) and isinstance(event, transaction.event):
                return transaction.next_state
        return None


class DVD(FSM):

    def __init__(self) -> None:
        super().__init__(None)


dvd = DVD()
dvd.add_transaction(DvdPowerOff, PowerOnEvent, DvdPowerOn)
dvd.add_transaction(DvdPowerOn, PowerOffEvent, DvdPowerOff)
dvd.add_transaction(DvdPowerOn, PlayEvent, DvdPlaying)
dvd.add_transaction(DvdPlaying, PowerOffEvent, DvdPowerOff)
dvd.add_transaction(DvdPlaying, PauseEvent, DvdPausing)
dvd.add_transaction(DvdPausing, PowerOffEvent, DvdPowerOff)
dvd.add_transaction(DvdPausing, PlayEvent, DvdPlaying)
dvd.run()

print('1' * 20)
dvd.process_event(PowerOnEvent())
print('2' * 20)
dvd.process_event(PlayEvent())
print('3' * 20)
dvd.process_event(PauseEvent())
print('4' * 20)
dvd.process_event(PlayEvent())
print('5' * 20)
dvd.process_event(PowerOffEvent())

"""
power off and dvd is power off
11111111111111111111
power on and dvd is power on
22222222222222222222
play and dvd is goding to play
33333333333333333333
stop play and dvd stop play
pause and dvd is going to pause
44444444444444444444
stop pause and dvd is stopped pause
play and dvd is goding to play
55555555555555555555
stop play and dvd stop play
power off and dvd is power off
"""
