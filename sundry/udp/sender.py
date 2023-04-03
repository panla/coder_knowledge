import socket
import time


class Sender:
    def __init__(self, ip: str, port: int, broadcast: bool = False, encoding: str = 'utf-8') -> None:
        self.addr = (ip, port)
        self.encoding = encoding

        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if broadcast:
            self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def work(self):

        while True:

            try:
                msg = f'time {time.time()} {self.addr[0]} {self.flag}'
                self.client.sendto(msg.encode(self.encoding), self.addr)
                print(msg)
            except Exception as exc:
                print(time.time(), exc)

            time.sleep(1)


if __name__ == '__main__':

    sender = Sender('192.168.9.255', 20000, True)

    sender.work()
