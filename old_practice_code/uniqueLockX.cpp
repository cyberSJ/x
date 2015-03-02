#include <mutex>
#include <thread>
#include <stdio.h>
#include <iostream>

/* Lesson learned:
 std::unique_lock needs std::mutex as an input.
 Once eaten the mutex, it can lock it or unlock it.
 The useufulness of unique_lock is that it unlocks the mutex
 when it destructs itself. So unlocking a mutex becomes automatic
 for unique_lock

 Two thread cannot possess the same mutex at the same time.
*/

// Create global mutex first
std::mutex mtx;

void print_block(int n, char c){
	// Create unique_lock mechanims for the mutex
	// so that we can use stdard out by ourself.
	std::unique_lock<std::mutex> lck(mtx);

	// Perform some function.
	for (int i = 0; i < n; i++){
		//printf("%c", c);
		std::cout << c;
		//lck.unlock();		// <-- calling this is an error because t1 and t2 are both trying to take ownership of the unique_lock and this operation is not permitted.
	}
	//printf("\n");
	std::cout << std::endl;

}

int main(int argc, char **argv){

	// Now, create thread each calling the same function.
	std::thread t1(print_block, 50, '*');
	std::thread t2(print_block, 50, '$');

	t1.join();
	t2.join();

	return 0;
}
