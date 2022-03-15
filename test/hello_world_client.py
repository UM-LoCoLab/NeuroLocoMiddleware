#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import zmq

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.PUSH)
socket.connect("tcp://localhost:5555")

socket2 = context.socket(zmq.PULL)
socket2.bind("tcp://*:5556")

#  Do 10 requests, waiting each time for a response
for request in range(10):
    print(f"Sending request {request} …")
    socket.send(b"Hello")

    while True:
        #  Get the reply.
        try:
            message = socket2.recv()
            print(f"Received reply {request} [ {message} ]")
            break
        except zmq.error.Again:
            pass

        time.sleep(0.001)