import socket
import time


class Sender:
    def __init__(self, ip: str, port: int, flag: bool) -> None:
        self.ip = ip
        self.port = port
        self.flag = flag

        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def work(self):

        while True:

            if self.flag:
                self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            try:
                msg = f'time {time.time()}'
                self.client.sendto(msg.encode('utf-8'), (self.ip, self.port))
                print(msg)
            except Exception as exc:
                print(time.time(), exc)

            time.sleep(1)


if __name__ == '__main__':
    # 服务端是 192.168.9.255:20000 True, ok
    # sender = Sender('192.168.9.255', 20000, True)

    # 服务端是 192.168.9.255:20000 True, no
    # 服务端是 192.168.9.96:20000 False, ok
    # sender = Sender('192.168.9.96', 20000, False)

    # 服务端是 192.168.9.255:20000 True, no
    # 服务端是 192.168.9.96:20000  True, ok
    # 服务端是 0.0.0.0:20000 False, ok
    # 服务端是 0.0.0.0:20000 True, ok
    sender = Sender('192.168.9.96', 20000, True)

    sender.work()

"""
发送方：
    局域网广播发送，接收方局域网广播接收，ok

    指定地址发送，接收方指定地址接收，ok

    发送方发送，接收方 0.0.0.0 接收，ok

"""
