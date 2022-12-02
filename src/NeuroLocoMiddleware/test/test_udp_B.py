#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#
from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
# import zmq
import socket
import numpy as np


print("B1")
# context = zmq.Context()

socketB = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)
print("B3")

#  Socket to talk to server
print("Connecting to hello world server…")
socketA = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)
# socketA.setsockopt(zmq.SUBSCRIBE, b'A')
print("B5")
# socketA.connect("tcp://localhost:5557")
socketA.bind(("127.0.0.1", 5557))
# print("B6")
# poller = zmq.Poller()
# poller.register(socketA, zmq.POLLIN)
print("B7")

my_count = 0

last_zero = 0.0
duration=-42

for t in SoftRealtimeLoop(0.0033, report=True):
    print("B8", t)
    # socks = dict(poller.poll(0))

    # print("B9", socks)
    # if socketA in socks and socks[socketB] == zmq.POLLIN:
    while True:
        try: 
            print("B10")
            message = socketA.recv(1024)
            print("B11 Received:", message)
            if int(message[1:4])==my_count:
                my_count+=1
                if my_count>=1000:
                    my_count=0
                    duration=t-last_zero
                    last_zero=t
        except BlockingIOError:
            break
    print("B12", my_count)

    vecB = np.array([[0.34,0.222, 0.5]])
    socketB.sendto(b"B%03d%s"%(my_count, vecB.tobytes()), ("127.0.0.1", 5558))
    print("B13", b"B%03d%s"%(my_count, vecB.tobytes()))
    print("B14", duration)