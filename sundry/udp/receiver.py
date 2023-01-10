import socket
import time


class Server:
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
                self.client.bind((self.ip, self.port))
            except Exception as exc:
                print(time.time(), exc)
                continue

            while True:
                try:
                    data, addr = self.client.recvfrom(1024)
                    print('收到', addr, data.decode('utf-8'))
                except Exception as exc:
                    print(time.time(), exc)


if __name__ == '__main__':

    # 发送方是 192.168.9.255:20000, True, ok
    # server = Server('192.168.9.255', 20000, True)

    # 发送方是 192.168.9.255:20000, True, no
    # server = Server('192.168.9.96', 20000, False)

    # 发送方是 192.168.9.96:20000, False, ok
    # server = Server('192.168.9.96', 20000, False)

    # 发送发是 192.168.9.96:20000  True, ok
    # server = Server('192.168.9.96', 20000, True)

    # server = Server('0.0.0.0', 20000, False)

    server = Server('0.0.0.0', 20000, True)

    server.work()
