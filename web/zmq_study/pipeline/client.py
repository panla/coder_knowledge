import zmq

conext = zmq.Context()
socket = conext.socket(zmq.PUSH)
socket.bind('tcp://*:5557')

for i in range(10):
    data = f'hello {i}'.encode()

    socket.send(data)
