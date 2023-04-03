import socket
import time


class Server:
    def __init__(
            self,
            ip: str,
            port: int,
            broadcast: bool = False,
            buff_size: int = 1024,
            encoding: str = 'utf-8'
    ) -> None:
        self.addr = (ip, port)
        self.buff_size = buff_size
        self.encoding = encoding

        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if broadcast:
            self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def work(self):

        while True:

            try:
                self.client.bind(self.addr)
            except Exception as exc:
                print(time.time(), exc)
                continue

            while True:
                try:
                    data, addr = self.client.recvfrom(self.buff_size)
                    print('收到', addr, data.decode(self.encoding))
                except Exception as exc:
                    print(time.time(), exc)


if __name__ == '__main__':

    server = Server('192.168.9.96', 20000, True)

    server.work()
