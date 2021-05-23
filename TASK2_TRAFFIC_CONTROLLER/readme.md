ALGORITHM:
At any point of time, if the given queue has 2 or more sides waiting to go right. Set the right signal on for both of them. If not 2 then look for the straight signal by the side next to it in anticlockwise fashion. If not this look for number of sides waiting for straight signal.If any of the pair (A,B) or (C,D) satisfies this, give them the signal.


MAIN CLASS:
STATE

SUB STATES:
stop
straight
right
straight and right

INSTANCES:
A
B
C
D

Initially and after each transition the states return back to stop signal.
This is done by the function execution_stop()which closes each of the 12 lights after each time step.

Function execution switches on the corresponding light  and decreases the queue by 1.

The function deleter and deletes removes the instances whose the queue number for right and straight going cars is zero from the array_right and array_straight respectively.



***HOW TO RUN ***
FIRST RUN TASK2.PY
THEN RUN CLIENT2.PY FROM CMD TO GIVE THE INSTRUCTIONS




