ALGORITHM:
At any point of time, if the given queue has 2 or more sides waiting to go right. Set the right signal on for both of them. If not 2 then look for the straight signal by the side next to it in anticlockwise fashion. If not this look for number of sides waiting for straight signal.If any of the pair (A,B) or (C,D) satisfies this, give them the signal.

STATES:
stop
straight
right

INSTANCES:
A
B
C
D









