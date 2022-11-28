import zmq

context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.bind("tcp://*:5558")

while True:
    data = socket.recv()
    print(data)
