import zmq

from interface1 import *
from message import *


registry = MessageRegistry()
registry.add(TestS1())
registry.add(TestS2())

context = zmq.Context()
print("Connecting to server")
socket = context.socket(zmq.PAIR)
socket.bind("tcp://localhost:5555")

# send first message
msg1 = TestS1()
msg1.i1 = 42
msg1.f1 = 3.14
msg1.b1 = False
msg1.u1 = 89

send_message(socket, msg1)

# send second message
msg2 = TestS2()
msg2.d1 = -3.14
msg2.i1 = -888

send_message(socket, msg2)

# receive messages
rec_msg1 = recv_message(registry, socket)
if isinstance(rec_msg1, TestS2):
    print("Received TestS2")
    print(rec_msg1)

rec_msg2 = recv_message(registry, socket)
if isinstance(rec_msg2, TestS1):
    print("Received TestS1")
    print(rec_msg2)
