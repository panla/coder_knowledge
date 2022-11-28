import zmq

context = zmq.Context()

socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, '')

for request in range(10):
    #  Get the reply.
    message = socket.recv()
    print(message)
