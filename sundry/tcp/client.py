import socket
import threading
import time


class TCPClient:
    def __init__(self, host: str, port: int, buff_size: int = 1024, encoding: str = 'utf-8') -> None:
        self.addr = (host, port)
        self.buff_size = buff_size
        self.encoding = encoding

        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_flag = False

    def connect(self):

        try:
            self.client.connect(self.addr)
            self.connect_flag = True
        except Exception as exc:
            print(f'{time.time()} {exc}')
            self.connect_flag = False

    def send_msg(self, data):

        if self.connect_flag:

            self.client.send(data)

            data = self.client.recv(1024)
            print(f'{time.time()} 接收到 ', data.decode(self.encoding))

    def receive_msg(self):

        while True:
            if self.connect_flag:

                data = self.client.recv(self.buff_size)
                print(f'{time.time()} 接收到 ', data.decode(self.encoding))

    def work(self):

        self.connect()

        # 起另一个线程，会丢数据
        # thread = threading.Thread(target=self.receive_msg)
        # thread.setDaemon(True)
        # thread.start()

    def close(self):
        self.client.close()


if __name__ == '__main__':
    tcp_client = TCPClient('192.168.9.96', 8750)
    tcp_client.work()

    time.sleep(2)

    for i in range(100):
        tcp_client.send_msg(f'走你 {i}'.encode('utf-8'))

    tcp_client.close()
