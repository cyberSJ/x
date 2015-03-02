#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <future>
#include <functional>

/* Lesson learned: std::thread::detach()
 If current thread needs to terminate before another thread that was 
 created within current thread, use detach().
 i.e. detach() allows your current thread to terminate safely without
 any waiting for the child thread to finish.

 Use join() if you want to wait for the child thread to finish.

 If you use neither of those, you'll get error.

 Two-way communication:
 ----------------------------
 If cv.wait() in thread2 happens before cv.notify_one() in thread1,
 thread2 will hang.
 class member variables are shared even though a new thread inside
 a class is created.

 In order for the cv.wait() to pass,
 The mutex should be in released state when the lock is first CREATED 
 with that mutex. I.e. when lock is created, the mutex should not be 
 owned by any other thread. I.e. lock must acquire "clean" mutex.

 In order for the cv.wait() to pass (more accurate than above),
 the lock used in cv.wait() must be able to acquire the mutex at
 the line cv.wait(). Upon exiting a scope, the lock will release the
 mutex, so that other thread doing cv.wait() on the same mutex can
 now execute.

 Make sure cv.notify_one() (or _all()) call is "heard" by the thread
 that does cv.wait(). For example, stdd::this_thread::sleep_for()
 really makes the thread sleep and IGNORE the noitfy call from other
 thread.

 When two mutually exclusive thread shares a data, it might appear
 that only one thread is working (because they really are concurrent
 occurring at the same time!).

 cv.notify_all() notifies only one kind of lock. Even if thread #1's
 cv.wait() and thread #2's cv.wait() use same condition_variable,
 if the mutexes used are different instances, cv.notify_all() does
 not notify all (at least not all the time).

 Once the std::promise is set, it cannot be reset.

 It takes about 24 us to create one thread.

 Don't use parenthesis after the function used in std::thread creation.

 Make sure you don't confuse parenthesis with brackets .

 Passing std::ref() of std::shared_ptr will pass null.

*/

void independentThread(){
	std::cout << "Starting independent thread\n";
	std::this_thread::sleep_for(std::chrono::seconds(2));
	std::cout << "Exiting independent thread\n";
}

void threadCaller(){
	std::cout << "Starting thread caller\n";
	std::thread it(independentThread);
	//it.detach();	// w/o detach(), independentThread goes out of scope, so it can't function properly. Results in "terminate called without an active exception" error.
	std::this_thread::sleep_for(std::chrono::seconds(4));
	std::cout << "Exicting thread caller\n";
	//it.join();	// Use this only if you want wait for independentThread to finish AND THEN finish threadCaller. If you use join(), your current thread's termination depends on other thread.
}

// Using a class's function as a thread function.
class ThreadClass{
public:
	ThreadClass(){}

	void showInt(){
		std::cout << 4 << std::endl;
	}

	void makeThread(){
		std::thread newThread(&ThreadClass::showInt, this);
		newThread.join();
	}

};

// Two-way communication between threads:
//--------------------------------------------
/*
	Scenario: main class runs, assign some one for slave thread,
	the slave thread returns (without terminating), and the
	main class continues to run. Useful for not creating
	a new thread every time I need parallelization.
*/
class TwoWay{
	int m_int;
	int m_int2;
	
	std::packaged_task<int ()> m_slave;

	std::condition_variable m_cvSlave;
	std::condition_variable m_cvSlave2;
	std::condition_variable m_cv;
	std::mutex m_mtx;
	std::mutex m_mtx2;
	std::mutex m_mtx3;
	std::mutex m_mtx4;

	std::promise<int> m_p1;
	std::promise<int> m_p2;
	std::future<int> m_f1;
	std::future<int> m_f2;


public:
	TwoWay() : m_int(0), m_int2(0), m_f1(m_p1.get_future()), m_f2(m_p2.get_future()){
	//TwoWay() : m_int(0), m_slave(std::bind(&TwoWay::work, this,5)) {//, m_f1(m_slave.get_future()) {
		//std::thread slave(&TwoWay::work, this, std::ref(m_mtx2), std::ref(m_cvSlave), std::ref(m_p1), 1);
		//slave.detach();
		//std::thread slave2(&TwoWay::work, this, std::ref(m_mtx2), std::ref(m_cvSlave2), std::ref(m_p2), 2);
		//slave2.detach();
	}


	void command(){
		// At this moment, work() is acquiring empty mutex.
		std::this_thread::sleep_for(std::chrono::seconds(1));
		// now we can do anything with the mutex that was
		// acquired in work() because m_cv.wait() in work()
		// just released the mutex.
		// It is better to use separate mutex for each
		// condition_variable.
		//std::unique_lock<std::mutex> lck(m_mtx);
		//std::unique_lock<std::mutex> lck4(m_mtx4);
		//std::cout << "commanding from master...\n";
		//m_cv.notify_all();												// make sure notify_all() happens AFTER wait() in other thread.

		//m_f1.wait();
		//m_f2.wait();

		// Using condition_variables
		//m_cvSlave.wait(lck);	
		//m_cvSlave2.wait(lck);
		//std::this_thread::sleep_for(std::chrono::seconds(1));
		//std::cout << "received slave work1. " << m_f1.get() << std::endl;
		//std::cout << "received slave work2. " << m_f2.get() << std::endl;

		// Using packaged_task
		//std::future<int> result = m_slave.get_future();
		//m_slave();
		//std::cout << "received slave work1. " << result.get() << std::endl;

		// Async
		//auto ret1 = std::async(std::launch::async, &TwoWay::work, this, 1);
		//auto ret2 = std::async(std::launch::async, &TwoWay::work, this, 2);
		//std::cout << "received slave work1. " << ret1.get() << std::endl;
		//std::cout << "received slave work1. " << ret2.get() << std::endl;

	}

private:
	//void work(std::mutex& mtx, std::condition_variable& cv, std::promise<int>& p, int id, std){
	int work(int id){
		// Using condition_variable
		//std::cout << "slave thread alive...\n";
		//std::unique_lock<std::mutex> lck(mtx);
		//while(1){
		//	{
		//		m_cv.wait(lck);
		//		std::cout << "slave #" << id << " working...\n";
		//	}
		//	m_int++;
		//	//p.set_value(1);
		//	//std::cout << "slave work result: " << gint << std::endl;
		//	//cv.notify_one();	
		//}

		// Using packaged_task or async
		//std::cout << "slave #" << id << " working...\n";
		//id++;
		//return id;
	}

};

// cppreference.com example:
//--------------------------------
#if 0
std::condition_variable cv;
std::mutex cvm;

int i = 0;

void waits(){
	std::unique_lock<std::mutex> lk(cvm);
	std::cerr << "Waiting... \n";
	cv.wait(lk, []{return i==1;});
	std::cerr << "...fininshed waiting. i ==1 \n";
}

void signals(){
	std::this_thread::sleep_for(std::chrono::seconds(1));
	{
		std::lock_guard<std::mutex> lk(cvm);
		std::cerr << "Notifying...\n";
	}
	cv.notify_all();

	std::this_thread::sleep_for(std::chrono::seconds(1));
	{
		std::lock_guard<std::mutex> lk(cvm);
		i = 1;
		std::cerr << "Noitfying again... \n";
	}
	cv.notify_all();
}
#endif

int main (int argc, char **argv){
	//threadCaller();
	//std::this_thread::sleep_for(std::chrono::seconds(5));
	//std::cout << "Exiting the main loop\n";

	//ThreadClass tc;
	//tc.makeThread();

	// Two-way communication:
	//--------------------------
	TwoWay twoWay;
	//std::this_thread::sleep_for(std::chrono::seconds(1));
	//twoWay.command();
	//twoWay.command();
	
	
	// cppreference.com example:
	//-----------------------------
	//std::thread t1(waits), t2(waits), t3(waits), t4(signals);
	//t1.join();
	//t2.join();
	//t3.join();
	//t4.join();

	return 0;
}
