import socket
import threading
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
        self.right_straight=right_straight(name)
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
        self.key="ON"
        self.name=name 
    def execute(self):
        self.key="OFF"
        #print("The stop signal is turned {}now for {}".format(self.key,self.name))
    def initial_state(self): #to reset the states
        self.key="ON"
             
class straight(state):  
    def __init__(self,name):
        self.key="OFF"
        self.name=name
        self.stop=stop(name)
    def execute(self):
        self.key="ON"
        print("{}---straight signal is on".format(self.name))
    def initial_state(self): #to reset the states
        self.key="OFF"                  

class right(state):  
    def __init__(self,name):
        self.key="OFF"
        self.name=name    
    
    def execute(self):
        self.key="ON"
        print("{}---right signal is on".format(self.name)) 
    def initial_state(self):   #to reset the states
        self.key="OFF"    

class right_straight(state):
    def __init__(self, name):
        self.key="OFF"
        self.name=name
    def execute(self):
        self.key="ON"
        print("{}---both right and straight signal is on".format(self.name)) 
    def initial_state(self):   #to reset the states
        self.key="OFF"                
       
def execution(a1,a2,step1,step2,step3,step4):
    step1-=1
    step2-=1
    step3.execute()
    step4.execute()   
    a1.stop.execute()
    a2.stop.execute() 
    return (step1,step2)

def execution_stop(array):
    for item in array:
        item.stop.initial_state()
        item.right.initial_state()
        item.straight.initial_state()
        item.right_straight.initial_state()

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
    
    not_done=True
    count_r=0
    count_s=0
    iters=1
    i=1
    array_right=[]
    array_straight=[]
    while not_done:
        print("Initially and after all the transitions all the 4 directions have a stop signal")        
        print("iteration{}".format((i)))         
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
        print("The initial Queue of iteration {}".format(i))  ##printing the initial queue
        for item in Traffic_System: 
            print(item.straight_,item.right_,end=" ") 
        print("\n")
        flag=0  
        if(count_r >= 2):  ##checking for number of right going sides
           (array_right[0].right_,array_right[1].right_)=execution(array_right[0], array_right[1],array_right[0].right_,array_right[1].right_,array_right[0].right,array_right[1].right)

           flag=1
           count_r,array_right=deleter(array_right,count_r)
                 
        elif(count_r==1):
            index=(Traffic_System.index(array_right[0])+2)%4
            if(Traffic_System[index].straight_>0):
                (array_right[0].right_,Traffic_System[index].straight_)=execution(array_right[0],Traffic_System[index],array_right[0].right_,Traffic_System[index].straight_,array_right[0].right,Traffic_System[index].straight)
                count_r,array_right=deleter(array_right,count_r)
                count_s,array_straight=deletes(array_straight,count_s)
                flag=1
            elif(array_right[0].straight_>0):
                (array_right[0].right_,array_right[0].straight_)=execution(array_right[0],array_right[0],array_right[0].right_,array_right[0].straight_,array_right[0].right_straight,array_right[0].right_straight)
                count_r,array_right=deleter(array_right,count_r)
                count_s,array_straight=deletes(array_straight,count_s)
                flag=1    
            else:
                flag=0
        if(flag==0 and count_r<=1):
            if(count_s>=2):
                if A in array_straight and B in array_straight:
                    (A.straight_,B.straight_)=execution(A,B,A.straight_,B.straight_, A.straight,B.straight)
                    count_s,array_straight=deletes(array_straight,count_s)
                    flag=1
    
                elif C in array_straight and D in array_straight:
                    (C.straight_,D.straight_)=execution(C,D,C.straight_,D.straight_, C.straight,D.straight)
                    count_s,array_straight=deletes(array_straight,count_s)
                    flag=1
                    
                else:
                    flag=0
                    
            if(flag==0 and count_s<=2):
                if(count_s!=0):
                    array_straight[0].straight_-=1
                    array_straight[0].straight.execute()
                    array_straight[0].stop.execute()
                    count_s,array_straight=deletes(array_straight,count_s)
                    flag=1
                    
                else:
                    if(count_r!=0):
                        array_right[0].right_-=1
                        array_right[0].right.execute()
                        array_right[0].stop.execute()
                        count_r,array_right=deleter(array_right,count_r)
                    else:    
                        not_done=False
        num_zero=0                
        print("Current states")
        for item in Traffic_System:
            print("{}--stop={},right={},straight={},right_And_straight={}".format(item.name,item.stop.key,item.right.key,item.straight.key,item.right_straight.key))
        print("The final Queue of iteration {}".format(i))
        for item in Traffic_System: 
            print(item.straight_,item.right_,end=" ")
            if(item.straight_==0):
                num_zero+=1
            if(item.right_==0):
                num_zero+=1      
        print("\n")    
        execution_stop(Traffic_System)  #to bring all the signals back to their inital state
        if(num_zero==8):
                not_done=False  
        i+=1                
                   

 
       
        



