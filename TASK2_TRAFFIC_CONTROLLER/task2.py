import fysom 
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
s= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("socket created")
try:
    s.bind(ADDR)

except socket.error as e:
    print(str(e))

class state:
    def __init__(self,name):
        self.straight_=0
        self.right_=0
        self.stop=stop(name)
        self.straight=straight(name)
        self.right=right(name)
        self.name=name
def deleter(array_r,count_r):
    indices=[]
    for x in range(len(array_r)):
        if array_r[x].right_==0:
            indices.append(x)
    count_r-=len(indices)
    iters=0

    while iters<len(indices):
        i=indices[iters]-iters
        array_r.remove(array_r[i])
        iters+=1

                

    return (count_r,array_r)       
def deletes(array_s,count_s):
    indices=[]
    for x in range(len(array_s)):
        if array_s[x].straight_==0:
            indices.append(x)
    count_s-=len(indices)
    iters=0

    while iters<len(indices):
        i=indices[iters]-iters
        array_s.remove(array_s[i])
        iters+=1         
            
    return (count_s,array_s)        


class stop(state):  
    def __init__(self,name):
        self.key={1:"ON"} 
        self.name=name  

class straight(state):  
    def __init__(self,name):
        self.key={0:"OFF"}
        self.name=name
    def execute(self):
        self.key={1:"ON"}
        print("{}---straight signal is on".format(self.name))    

class right(state):  
    def __init__(self,name):
        self.key={0:"OFF"}
        self.name=name
    def execute(self):
        self.key={1:"ON"}
        print("{}---right signal is on".format(self.name))  


if __name__=='__main__':
    A=state("A")
    B=state("B")
    C=state("C")
    D=state("D")
    Traffic_System = []

    Traffic_System.append(A)
    Traffic_System.append(B)
    Traffic_System.append(C)
    Traffic_System.append(D)
    print("waiting for connections")
    s.listen(1)
    print(f"[LISTENING] Server is listening on {SERVER}")
    c, add = s.accept()
    x=int(pickle.loads(c.recv(1024)))
    
  #  x=int(input("Enter the number of time_steps"))
    
    not_done=True
    count_r=0
    count_s=0
    iters=1
    i=1
    array_right=[]
    array_straight=[]
    while not_done:
        
        
        print("iteration{}".format((i)))
         
        print("Initially all the 4 directions have a stop signal")
        if iters<x+1:
            a=pickle.loads(c.recv(1024))
            init=0
            iters+=1
            for items in Traffic_System:
                items.straight_+=int(a[2*init])
                items.right_+=int(a[2*init+1])
                
                if(items.right_==1 and int(a[2*init+1])!=0):
                    count_r+=1
                    array_right.append(items)
                if(items.straight_==1 and int(a[2*init])!=0):  
                    count_s+=1
                    array_straight.append(items) 
                init+=1     
        print("The initial Queue of iteration {}".format(i))
        for item in Traffic_System: 

            print(item.straight_,item.right_,end=" ") 
        print("\n")
        flag=0     
        if(count_r >= 2):
           array_right[0].right_ -= 1
          
           array_right[1].right_-= 1
    
           array_right[0].right.execute()
           array_right[1].right.execute()
           flag=1
           count_r,array_right=deleter(array_right,count_r)
                 
        elif(count_r==1):
            index=(Traffic_System.index(array_right[0])+2)%4
            if(Traffic_System[index].straight_>0):
                array_right[0].right_-=1
                Traffic_System[index].straight_-=1
                array_right[0].right.execute()
                Traffic_System[index].straight.execute()
                count_r,array_right=deleter(array_right,count_r)
                count_s,array_straight=deletes(array_straight,count_s)
                flag=1
                
            else:
                flag=0



        elif(flag==0 or count_r==0):
            if(count_s>=3):
                if A in array_straight and B in array_straight:
                    A.straight_-=1
                    B.straight_-=1
                    A.straight.execute()
                    B.straight.execute()
                    count_s,array_straight=deletes(array_straight,count_s)
                    flag=1
    
                else:
                    C.straight_-=1
                    D.straight_-=1
                    C.straight.execute()
                    D.straight.execute()
                    count_s,array_straight=deletes(array_straight,count_s)
                    flag=1
                    
            elif(count_s==2):
                if A in array_straight and B in array_straight:
                    A.straight_-=1
                    B.straight_-=1
                    A.straight.execute()
                    B.straight.execute()
                    count_s,array_straight=deletes(array_straight,count_s)
                    flag=1
                   
                elif C in array_straight and D in array_straight:
                    C.straight_-=1
                    D.straight_-=1
                    C.straight.execute()
                    D.straight.execute()
                    count_s,array_straight=deletes(array_straight,count_s)
                    flag=1
                    
                else:
                    array_straight[0].straight_-=1
                    array_straight[0].straight.execute()
                    count_s,array_straight=deletes(array_straight,count_s)
                    



            else:
                if(count_s!=0):
                    array_straight[0].straight_-=1
                    array_straight[0].straight.execute()
                    count_s,array_straight=deletes(array_straight,count_s)
                    
                else:
                    if(count_r!=0):
                        array_right[0].right_-=1
                        array_right[0].right.execute()
                        count_r,array_right=deleter(array_right,count_r)
                    else:    
                        not_done=False
        print("The final Queue of iteration {}".format(i))
        for item in Traffic_System: 
            print(item.straight_,item.right_,end=" ") 
        print("\n") 
        i+=1                
                   













            
        
       
        



