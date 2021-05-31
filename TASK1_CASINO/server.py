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
    num_arr= [[0 for x in range(3*k)] for y in range(r)]
    new_arr= [[0 for x in range(3*k)] for y in range(r)] 
     
    for i in range(r):
        for j in range(3*k):
            num_arr[i][j]=randint(1, 52)
        np.random.shuffle(num_arr[i])
        new_arr[i] = scores(num_arr[i])
    return new_arr
def initialise(r,k,t):
    global score_b

    while(len(score_b)>0):
        score_b=np.delete(score_b,0,axis=0)
    dict1={}        
    return (score_b,dict1) 
a=[]         
def result_board(name,value,r,k):
    global a,score_b,dict

    for i in range(r):
        #print(value[0],i+1)
        if(value[0]==i+1):
            print(" round {} ".format(i+1))
            #print(score_b)
            a.append(value[1])
            #score_b[i].append(value[1])
            #empty_array = np.append(score_b[i], np.array([value[1]]), axis=0)
            if(len(a)==k):
                score_b=np.append(score_b,np.array([a]),axis=0)
                #print(score_b)
                high_score=max(score_b[i])
                #print(high_score)
                winner=list(dict.keys())[list(dict.values()).index((i+1,high_score))]
                print("Winner of round {} is {}".format(i+1,winner))
                dict1[winner]+=1
                a=[]

        if(value[0]==r and value[0]==i+1 and len(a)==0 ):
            #print(score_b)
            if( len(score_b[r-1])==k):
                print("winner of all rounds is  {}".format(max(dict1, key=dict1.get)))
names=[] 

a_n=[[]]
def shuffle_new(val, a):
    global a_n
    arr = []
    
    if(val==1):
        for i in range(len(a)):
            for j in range(len(a[i])):
                arr.append(a[i][j])
        
        a_n= [[0 for x in range(len(a[0]))] for y in range(len(a))] 
        np.random.shuffle(arr)
        #print("shuffled array {}".format(arr))
        
        k=0
        for i in range(len(a)):
            for j in range(len(a[i])):
                a_n[i][j]=arr[k]
                k+=1 
           
        return a_n   
    return a_n        
              

def handle_client(c,add,name,score,dict,arrw,client_num,R,k):
    
    connected = True
    name = c.recv(1024).decode(FORMAT)
    names.append(name)
    print("connected with", add,name)
    global dict1
    
    c.send(bytes("welcome to casino", 'UTF-8'))
    flag=1
    
    while connected:
        play = c.recv(1024).decode(FORMAT)
        set_of_rounds=0
        
        if play == DISCONNECT_MESSAGE:
            connected = False
        if play == "YES" or play == "NO":
            if play == "NO":
                connected = False
                flag=0
            if play == "YES":
                dict1[name]=0
                flag=1
                set_of_rounds+=1
               # print(set_of_rounds)
                
                arrw=shuffle_new(client_num,arrw)
               # print(arrw)    

                c.send(bytes("Thanks for playing!!", 'UTF-8'))
                r=1
                c.send(pickle.dumps(R))
                scoretotal=0
                while(r<R+1):
                    
                    arr=arrw[r-1][3*(client_num-1):3*client_num]
                   # print(arr)
                    time.sleep(1)
                    c.send(pickle.dumps(arr))
                    time.sleep(1)
                    score=int(pickle.loads((c.recv(1024))))
                    #print("client name ",name,score)
                    scoretotal+=score
                    dict[name]=(r,score)
                    print(dict)
                    result_board(name,dict[name],R,k)
                    time.sleep(1)
                    r+=1
                    
        global score_b    
        #print(score_b)     
        while(connected==True and len(score_b[0])<k):
            time.sleep(1) 
        print(dict1)            
        if( flag==1 and len(score_b[0])==k):
            c.send(pickle.dumps("The winner is{} and you won {} out of {} rounds with a score of {}.".format(max(dict1, key=dict1.get),dict1[name],R,scoretotal)))
            flag+=1
        
        time.sleep(5)
        c.send(bytes("The Game is Over.",FORMAT))
        #print("yes")
        #print(score_b)
        
        (score_b,dict1)=initialise(R,k,client_num)
       
    time.sleep(1)           
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
score_b=np.empty((0,k), int)
start(k,r)  

