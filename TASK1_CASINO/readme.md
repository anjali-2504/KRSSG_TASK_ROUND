ALGORITHM:
 First asked the client for the number of rounds and number of participants for the bonus round by creating a socket.
 Then created a while loop and used multithreading for the various clients with the target :handle_client function and several parameters..In order to maintain the 
 effective transfer of data through server and clients , I used time.sleep(1). For the transfer of non-string like objects ,I imported the pickle library.
 
 The array of random numbers is generated in the starting itself and stored in the new_arr.
 
 The handle_clients function receives the name from the client, response if he wants to play or not. If the response is NO, connected is set to False.If YES:
 the player is given the three numbers and then receives the highest val from the client which is then sent to another function: result_board which calculates the maximum for all
 clients and then displays the winner name on the server.
 1.The result_board prints the result only after all the clients have responded. This is ensured by keeping track of the length of the elements in the score_b for each round.
 2.The array generated through random numbers is shuffled properly to ensure that the game is unbiased.
 3.The server also displays the number of clients playing the game by using the threading.ThreadCount()
 4.The dictionary dict  keeps record of the name of the player along with the round and his score in that round.
 5.flag keeps a check on the fact that the message "YOU WON {} ROUNDS OUT OF {}" does not get displayed over and over again in the while loop.
 
 ***HOW TO RUN:
 run the server.py first
 Open the cmd and then run :
 python client.py
 In order to start the game enter 1 and then the required answers to the questions. Now, open as many terminals as you specified and now enter 0 to distinguish that you are the    player.Now, you can enjoy the game.***
 
 
 
