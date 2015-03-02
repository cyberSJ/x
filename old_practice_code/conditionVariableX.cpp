#include <thread>
#include <condition_variable>
#include <mutex>
#include <iostream>

/*	Lesson learned:
 condition_variable must be known by all threads.
 cv.notify_one() in one function (or another thread) 
 lets one cv in another thread to check its cv.wait() again.
 So..... cv.notify_one() triggers cv.wait()

*/

// global condition variable that threads wait for
std::condition_variable cv;

// global mutex that the thread can refer to
std::mutex mtx;

bool ready = false;

// function to thread upon. It waits for conditional variable.
void print(int i){
	// Obtain the lock set by someone else
	std::unique_lock<std::mutex> lck(mtx);
	//while(!ready) cv.wait(lck);
	//cv.wait(lck);
	cv.wait(lck, []{return ready;});	// wait until another thread holding the same mutex sets the "ready" variable to true;
	//cv.notify_one();

	std::cout << "Thread " << i << std::endl;
}

void go(){
	// Set the conditional variable so that the thread can begin
	std::unique_lock<std::mutex> mainLock(mtx);
	//ready = true;
	cv.notify_all();
}

int main(){
	// Create a thread
	//std::thread threads[10];
	std::thread t1(print, 4);
	//for (auto& th : threads) th = std::thread(print,4);
	//for (int i = 0; i < 10; ++i)
	//	threads[i] = std::thread(print, i);

	//go();
	{
		std::unique_lock<std::mutex> mainLock(mtx);
		ready = true;
		cv.notify_one();	// alert one cv to see its condition in cv.wait()
	}
	
	t1.join();
	//for (auto &th : threads ) th.join();


	return 0;
}
