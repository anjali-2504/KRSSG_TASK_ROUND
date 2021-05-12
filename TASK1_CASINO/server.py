import socket
from random import randint
import threading
import time
import numpy as np
import pickle
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = ('localhost', 9999)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
dict = {}
dict1={}
s= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("socket created")
try:
    s.bind(ADDR)

except socket.error as e:
    print(str(e))
r=1
k=1
def get_client_num():
    s.listen(1)
    c, add = s.accept()
    k=int(pickle.loads(c.recv(1024)))
    time.sleep(1)
    r=int(pickle.loads(c.recv(1024)))
    return (k,r)


def gen_arr(r,k):
    num_arr= [[0 for x in range(r)] for y in range(3*k)]
    new_arr= [[0 for x in range(r)] for y in range(3*k)] 
     
    for i in range(r):
        for j in range(3*k):
            num_arr[i].append(randint(1, 52))
        np.random.shuffle(num_arr[i])
        new_arr[i] = scores(num_arr[i])
    return new_arr


def result_board(name,value,r,k):

    for i in range(r):
        print("val ",value[0])
        if(value[0]==i+1):
            score_b[i].append(value[1])
            print(score_b[i])
            if(len(score_b[i])==2*k):
                high_score=max(score_b[i])
                winner=list(dict.keys())[list(dict.values()).index((i+1,high_score))]
                print("Winner of round {} is {}".format(i+1,winner))
                dict1[winner]+=1

        if(value[0]==r and len(score_b[r-1])==2*k):
            print("winner of all rounds is  {}".format(max(dict1, key=dict1.get)))
            
                    
    
  
names=[]
def handle_client(c,add,name,score,dict,arrw,client_num,R,k):
    
    connected = True
    name = c.recv(1024).decode(FORMAT)
    names.append(name)
    print("connected with", add,name)
    dict1[name]=0
    c.send(bytes("welcome to casino", 'UTF-8'))
    flag=1
    
    while connected:
        play = c.recv(1024).decode(FORMAT)
        if play == DISCONNECT_MESSAGE:
            connected = False
        if play == "YES" or play == "NO":
           
            if play == "NO":
                connected = False
            if play == "YES":
                
                c.send(bytes("Thanks for playing!!", 'UTF-8'))
                r=1
                c.send(pickle.dumps(R))
                while(r<R+1):
                
                    arr=arrw[r-1][3*(client_num-1):3*client_num]
                    print(arr)
                    time.sleep(1)
                    c.send(pickle.dumps(arr))
                    time.sleep(1)
                    score=int(pickle.loads((c.recv(1024))))
                    print("client name ",name,score)
                    dict[name]=(r,score)
                    result_board(name,dict[name],R,k)
                    time.sleep(1)
                    r+=1
                    print(dict)
                  
        while(len(score_b[R-1])<2*k):
            time.sleep(1)         
        if(len(score_b[R-1])==2*k and flag==1):
            c.send(pickle.dumps("The winner is{} and you won {} out of {} rounds.".format(max(dict1, key=dict1.get),dict1[name],R)))
            flag+=1
        


        c.send(bytes("The Game is Over",FORMAT))         
    c.close()
   

def scores(num_arr):
    new_arr=[]
    for x in num_arr:
        if(x%13!=0):
            x=x%13
            new_arr.append(x)
        else:
            x=13
            new_arr.append(x)   
    return new_arr        




def start(k,r):
    print("waiting for connections")
    s.listen(k)
    print(f"[LISTENING] Server is listening on {SERVER}")
    
    client_num = 1
    new_Arr=gen_arr(r,k)
    print(new_Arr)
    while True:
        name = None
        score = -1
        c, add = s.accept()
        thread = threading.Thread(target=handle_client, args=(c, add, name, score, dict,new_Arr, client_num,r,k))
        thread.start()
        c1 = threading.activeCount() - 1
        client_num += 1
        print(f"[ACTIVE CONNECTIONS] {c1}")

print("[STARTING] server is starting...")
(k,r)=get_client_num()
score_b= [[0 for x in range(k)] for y in range(r)] 
start(k,r)  
