import socket
import threading
import time


class handleClient(threading.Thread):

    def __init__(self, client, addr) -> None:
        super().__init__()

        self.client = client
        self.addr = addr

    def run(self):

        print(f'{time.time()} 获取一个连接 {self.addr}')

        while True:

            try:
                original_data = self.client.recv(1024)
                data = original_data.decode('utf-8')
            except Exception as exc:
                data = f"{exc}".encode('utf-8')
                print(f'{time.time()} 异常 {data}')

            print(f'{time.time()} 接收到 {data}')

            if not data:
                print(f'{time.time()} 本次无数据，关闭')
                break

            self.client.send(original_data)

    def __del__(self):
        print(f'{time.time()} 本次客户服务结束')
        self.client.close()


class TcpServer(threading.Thread):
    def __init__(self, address, port):
        super().__init__()

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        self.server.bind((address, port))
        self.server.listen(128)

    def run(self):
        while True:
            client, addr = self.server.accept()
            handleClient(client, addr).start()

    def __del__(self):
        print(f'{time.time()} 服务器关闭')
        self.server.close()


if __name__ == '__main__':
    server = TcpServer('0.0.0.0', 8750)
    server.start()
