import socket
import pickle
import time
DISCONNECT_MESSAGE = "!DISCONNECT"
FORMAT = 'utf-8'
c = socket.socket()
ADDR = ('localhost', 9999)
c.connect(ADDR)
print("connected")
x=input("Enter the number of time_steps")
c.send(pickle.dumps(x))
for i in range(int( x)):
    a=input("Enter the string")
    c.send((pickle.dumps(a)))
    