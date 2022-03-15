#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.bind("tcp://*:5555")
socket2 = context.socket(zmq.PUSH)
socket2.connect("tcp://localhost:5556")

# while True:
#     #  Wait for next request from client
#     message = socket.recv()
#     print(f"Received request: {message}")

#     #  Do some 'work'
#     time.sleep(1)

#     #  Send reply back to client
#     socket.send(b"World")

while True:
    #  Wait for next request from client
    try:
        message = socket.recv(zmq.NOBLOCK)
        print(f"Received request: {message}")

        #  Do some 'work'
        time.sleep(0.1)

        #  Send reply back to client
        socket2.send(b"World")
    except zmq.error.Again:
        pass

    time.sleep(0.001)