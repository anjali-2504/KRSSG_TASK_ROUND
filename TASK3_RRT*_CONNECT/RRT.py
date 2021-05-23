import cv2
import numpy as np
import math
import os 
import time
import random
class RRTMap:
    def __init__(self, start, goal, MapDimensions,img):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions


        self.map=img
      
        self.nodeRad = 2
        self.nodeThickness = 1
        self.edgeThickness = 1

        

        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)

    def drawMap(self):
        
        cv2.circle(self.map, self.start, self.nodeRad , self.Green ,self.nodeThickness)
        cv2.circle(self.map, self.goal, self.nodeRad , self.Red,self.nodeThickness)
        #print("circles")
        

    def drawPath(self, path):
        for node in path:
            cv2.circle(self.map, node,3*(self.nodeRad) , self.Green,2)
            #print(node)
           # print("circle drawn")

    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            cv2.rect(self.map, self.grey, obstacle)


obstacles = []
class RRTGraph:
    def __init__(self, start, goal, MapDimensions, img):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.x = []
        self.y = []
        self.parent = []
        self.dist_startnode=[]
        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        self.dist_startnode.append(0)
        # the obstacles



        # path
        self.goalstate = None
        self.path = []
        self.map=img
        #cv2.imshow("window",img)
        #print(self.obstacles)



    def add_node(self, n, x, y):
        self.x.insert(n,x)
        self.y.append(y)
        
        #print(len(self.x))
        #print("inserted")

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        print("NUMBER OF NODES {}".format(len(self.x)))
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)

    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def collidepoint(self,rect,x1,y1):
        if(x1>=rect[0]and x1<=rect[0]+rect[2]and y1>=rect[1] and y1<=rect[1]+rect[3]) :
            return True


    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = obstacles.copy()
        #print(len(obs))
        while len(obs) > 0:
            rectang = obs.pop(0)
            if self.collidepoint(rectang,x,y):
                self.remove_node(n)
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        obs = obstacles.copy()
        while (len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0, 200):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if self.collidepoint(rectang,x, y):
                    return True
        return False

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            if abs(x - self.goal[0]) <= dmax and abs(y - self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def new_parent(self,n,radius=6):
        nodes_of_concern=[]
        for i in range(n):
            if(self.distance(i,n)<radius):
                nodes_of_concern.append(i)
        nnear=self.nearest(n)
        min_dist=self.cost(nnear)    
        parent_node=nnear 
        for ids in nodes_of_concern:
            if(self.cost(ids)<min_dist):
                min_dist=self.cost(ids)
                parent_node=ids
        return parent_node        
    def rewiring(self,n,radius=6,flag=False):
        if(flag):
            nodes_of_concern=[]
            for i in range(n):
                if(self.distance(i,n)<radius):
                    nodes_of_concern.append(i)
            unwired_node=[]
            main_node=[]
            for ids in nodes_of_concern:
                if(self.cost(ids)>self.cost(n)+self.distance(ids,n)):
                   # K=self.parent[ids]
                    unwired_node.append(self.parent[ids])
                    main_node.append(ids)
                    
                    cv2.line(self.map,(self.x[ids],self.y[ids]),(self.x[self.parent[ids]],self.y[self.parent[ids]]),(0,0,0),1)
                     #mapthickness used here
                    self.remove_edge(ids)
                    self.connect(ids,n)
                    cv2.line(self.map,(self.x[ids],self.y[ids]),(self.x[n],self.y[n]),(0,0,255),1)
        
        
    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        #print(self.x,self.y)
        ids = self.new_parent(n)

        self.step(ids, n)
        self.connect(ids, n)
        #self.step(nnear, n)
        #self.connect(nnear, n)
        
        return (self.x, self.y, self.parent)

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            ids = self.new_parent(n)
            self.step(ids, n)
            self.connect(ids, n)
        return self.x, self.y, self.parent

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
            #print("path")
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def cost(self, n):
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.distance(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c

    def getTrueObs(self, obs):
        TOBS = []
        for ob in obs:
            TOBS.append(ob.inflate(-50, -50))
        return TOBS

    def waypoints2path(self):
        oldpath = self.getPathCoords()
        path = []
        for i in range(0, len(self.path) - 1):
            print(i)
            if i >= len(self.path):
                break
            x1, y1 = oldpath[i]
            x2, y2 = oldpath[i + 1]
            print('---------')
            print((x1, y1), (x2, y2))
            for i in range(0, 5):
                u = i / 5
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                path.append((x, y))
                print((x, y))

        return path




def main():

    iteration = 0
    t1 = 0

    img = cv2.imread("rrt.jpg")
    dimensions = img.shape[:2]
    #print("yes")
    cv2.imshow('window', img.astype(np.uint8))

    start = (574, 541)
    goal = (57, 52)
    

    lower = np.array([0, 0, 0])
    upper = np.array([0, 0, 255])
    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img2, lower, upper)
    output = cv2.bitwise_and(img, img, mask=mask)



    output_= cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    cv2.imwrite("output.jpg",output_)
    ret, thresh = cv2.threshold(output_, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh,  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    global obstacles

    map = RRTMap(start, goal, dimensions,img)
    graph = RRTGraph(start, goal, dimensions,img)
    for cnt in contours[:9]:
            (x, y, w, h) = cv2.boundingRect(cnt)
            obstacles.append((x,y,w,h))
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            #print(x,y,w,h)
    cv2.namedWindow("window")        
    cv2.imshow('window', img)        
    map.drawMap() 
    
    '''while iteration<300:
        x,y=graph.sample_envir()
        n=graph.number_of_nodes()
        print(n)
        graph.add_node(n,x,y)
        graph.add_edge(n-1,n)
        x1,y1=graph.x[n],graph.y[n]
        x2,y2=graph.x[n-1],graph.y[n-1]
        
        if(graph.isFree()):
            cv2.circle(img,(graph.x[n],graph.y[n]),map.nodeRad+1,map.Red,map.nodeThickness)
            if not graph.crossObstacle(x1,x2,y1,y2):
                cv2.line(img,(x1,y1),(x2,y2),map.Blue,1)
            print("drawn")
        iteration+=1
        cv2.imshow('window', img)   
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()'''




    #t1=time.time()
    #print(graph.path_to_goal())
    while (not graph.path_to_goal()):
        '''time.sleep(0.005)
        elapsed=time.time()-t1
        t1=time.time()
        #raise exception if timeout
        if elapsed > 10:
             raise Exception('timeout re-initiating the calculations')'''
        #print(iteration%10)    

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            print(X,Y)
            cv2.circle(img,  (X[-1], Y[-1]), map.nodeRad*2,map.grey, 0)
            cv2.line(img, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.Blue,
                             map.edgeThickness)
            cv2.imshow("window",img.astype(np.uint8))                 
           # print("done")                 

        else:
            X, Y, Parent = graph.expand()
            cv2.circle(img, (X[-1], Y[-1]), map.nodeRad*2,map.grey, 0)
            cv2.line(img,  (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),map.Blue,
                             map.edgeThickness)
            cv2.imshow("window",img.astype(np.uint8))                 

        if iteration % 5 == 0:
            graph.rewiring(graph.number_of_nodes()-1,flag=True)
            cv2.imshow("window",img.astype(np.uint8))
        iteration += 1  
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break 
    map.drawPath(graph.getPathCoords())
    cv2.imshow("window",img.astype(np.uint8))
    #img2=img.copy()
    #cv2.circle(img, (506,548),6, (255,255,255),5)
    #print("yes")
    time.sleep(5)
    cv2.waitKey(0)
    cv2.destroyAllWindows() 


if __name__ == '__main__':
    '''result=False
    while not result:
        try:
            main()
            result=True
        except:
            result=False'''
    main() 
