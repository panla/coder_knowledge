import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

while True:
    data = b"Hello"

    time.sleep(1)

    socket.send(data)
