import socket
import pickle
import time
DISCONNECT_MESSAGE = "!DISCONNECT"
FORMAT = 'utf-8'
c = socket.socket()
ADDR = ('localhost', 9999)
c.connect(ADDR)
def send(msg):
    message = msg.encode(FORMAT)
    c.send(message)
    print(c.recv(1024).decode(FORMAT))
    print()
def ask_client_num():
    num_=input("Enter the number of players")
    rounds=input("Enter the number of rounds")
    c.send(pickle.dumps(num_))
    c.send(pickle.dumps(rounds))
bool_ans=0    
bool_ans=int(input("enter 1 if you started the game else enter 0"))
if bool_ans == 1:
    ask_client_num()
else:
    name = input("Enter your name..")
    send(name)
    play = input("do you want to play the game? YES OR NO")
    send(play)
    r=int(pickle.loads(c.recv(1024)))
    for i in range(r):
        print(pickle.loads(c.recv(1024)))
        num = input("Enter max val")
        c.send(pickle.dumps(num))
        print("Thanks!! Time for the next round..")
        time.sleep(1)
    time.sleep(5)
    print(pickle.loads(c.recv(1024)))
    c.recv(1024).decode()
send(DISCONNECT_MESSAGE)


