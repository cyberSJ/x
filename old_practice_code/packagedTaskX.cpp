#include <future>
#include <thread>

/* Lesson learend:
 packaged_task allows one to execute a function, dump the result of
 the function into std::future object (but not retrieve the result
 in the main program for now), and retrieve the result from the 
 std::future object in the main program whenever the user wants it.

 Trying to retrieve the result before task even starts hangs the 
 program.
*/

int myFunction(int a, int b){
	return a + b;
}

int main (int argc, char **argv){
	// Set up a packaged task
	std::packaged_task<int (int, int)> tsk(myFunction);

	// Get ready to obtain future result of the packaged task
	std::future<int> fut = tsk.get_future();
	
	// Try to get the result even before the task starts 
	//printf("result %d\n", fut.get());	// <-- it will block becuase the task is not even started.

	// Not execute the packaged task in a thread.
	std::thread th (std::move(tsk), 1, 3);

	// Get the result when the packaged task finishes
	printf("result %d\n", fut.get());
	
	// Don't forget to wait for the thread to finish.
	// But the packaged task already finished inside the thread.
	// So thread is still alive until this point, but the task
	// inside the thread is finished, so we can get the result
	// even though thread is not finished.
	th.join();
	return 0;
}
