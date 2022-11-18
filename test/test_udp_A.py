from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
# import zmq
import time
import socket
import numpy as np
#∅mq
#ØMQ
#🎄
print("A1")
# context = zmq.Context()
# print("A2")
socketA = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)

print("A3")
# socketA.bind("tcp://*:5557")
# print("A4")
socketB = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)

print("A5")
socketB.bind(("127.0.0.1", 5558))

# socketB.setsockopt(zmq.SUBSCRIBE, b'B')
# print("A6")
# poller = zmq.Poller()
# poller.register(socketB, zmq.POLLIN)

print("A7")
my_count = -42
last_zero = 0.0
duration=-42

for t in SoftRealtimeLoop(0.0033, report=True):
    print("A8", t)
    # socks = dict(poller.poll(1))

    # print("A9", socks)
    # if socketB in socks and socks[socketB] == zmq.POLLIN:
    print("A10")
    n_messages = 0
    while True:
        try: 
            message = socketB.recv(1024)
            print("A11")
            print("Received:", message)
            my_count=int(message[1:4])
            my_vecB = np.frombuffer(message[4:])
            print("A12", my_count, my_vecB)
            n_messages+=1
        except BlockingIOError:
            break
    print("A12.5, nmessages", n_messages)
    # if n_messages>=1:
        #  Send reply back
    socketA.sendto(b"A%03d"%my_count, ("127.0.0.1", 5557))
    print("A13", b"A%03d"%my_count)
    if my_count==0:
        duration=t-last_zero
        last_zero=t
    print("A14", duration)