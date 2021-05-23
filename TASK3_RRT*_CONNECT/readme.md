
PART -1:
TO IMPLEMENT THE RRT* CONNECT ALGORITHM

There are two files RRT.PY AND RRT*_CONNECT.PY
The first file is the basis implementation of RRT algorithm.
_The second file is the basic implementation of the RRT*CONNECT ALGORITHM


FUNCTIONS USED in classes (RRTGRAPH AND RRT MAP) :
BASIC FUNCTIONS:
add node
remover node
add edge
remove edge
distance (for calculating the distances between two nodes of a tree)
sample_envr(to genarate random points in the space)
path  to goal :which stores the coordinates of the points of the path after the path is found

RRT* IMPLEMENTATION FUNCTIONS
rewiring : to rewire the tree after every 5 iterations within a certain node radius
new_parent: to find the parent of new node using heuristic values

FUNCTIONS CALLED IN MAIN:
bias after every 10th iteration  and function expand with almost the same purpose of expanding the tree.
These functions return  the x and y coordinate of the new node and the parent which are then shown show using opencv cv2.line method.The nodes 
is marked using cv2.circle method.

FOR CONNECT ALGORITHM :
function check graph is created which will check when the two rewired trees meet each other and while loop is terminated.

DRAW PATH FUNCTION:
This function receives the list of coordinates from the path_to_goal function and then marks the nodes using white colour.

when you run the program you see..

Two trees start one from green node ,the other from red node with different colours and then some blue nodes in between
which show that rewiring has been done in the tree.
Also, the obstacles are marked using cv2.rectangle function of opencv which creates a rectangle around each obstacle. 
The white portion of image 1 are extracted using HSV colour extraction method whereas the contours are now create using findcontours function which provides the 
coordinates of the end point of the rectangle.

In case of image 2, we need to use a different HSV value range that is the range of grey colours.The file contours.py shows the contours and then similarly this 
image can be used to perform RRT* connect algorithm.

Sometimes the error:list range not in index may be encounterd . like 1 out of 10 times you run the file.This could be handled using error handling.
The code given below when pasted in the def __init__=='__main__' would resolve the issue:
result=False
    while not result:
        try:
            main()
            result=True
        except:
            result=False


*****THE CODE GIVEN BELOW
    a=graph.getPathCoords()
    b=graph1.getPathCoords()
    a1=[]
    a=a[::-1]
    for f in a :
        a1.append(f)
    for f in b :
        a1.append(f)*****
        
        
        
    THIS CODE AT THE END OF MAIN()STORES AND PRINTS THE PATH COORDINATES    .IF WE COLLECT THESE COORDINATES THEN THESE CAN BE USED IN THE TURTLESIM SIMULATOR 
    TO RUN THE TURTLE ON THE PATH GENERATED.
    
    
PART 2: RUNNING THE TURTLESIM SIMULATOR

There are many functions  in this:
 def move to move the turtle to a specified distance.
 def go_to_goal which makes use of the proportional linear and angular controller. 
 ***could not get time to implement derivative and integral controller***
 def rotate which would rotate the turtle when the motion is not just vertical or not just horizontal
 ***The class PID ()though not completed fully would display the error and the feedback for PID portions of the controller.***
 def set desired orientation is actually used to instruct which direction would the turtle currently point to clockwise or anticlokwise based on the relative 
 value of angle
 
 Thsi file could be executed by:
***$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/tutorial_ws/src
$ cd ~/tutorial_ws
$ catkin_init_workspace src
$ catkin_make
$ source devel/setup.bash
$ cd %YOUR_CATKIN_WORKSPACE_HOME%/src
$ catkin_create_pkg learning_tf tf roscpp rospy turtlesim
 $ cd %YOUR_CATKIN_WORKSPACE_HOME%/
 $ catkin_make
 $ source ./devel/setup.bash
 $ roscd learning_tf
 $ mkdir nodes
  $ chmod +x nodes/turtlesim_simulator.py
  new terminal
   $roscore
  new terminal
  $ rosrun turtllesim turtlesim_node
first terminal
  $ rosrun learning_tf turtlesim_simulator.py***
 

    





