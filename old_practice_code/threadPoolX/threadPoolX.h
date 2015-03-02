#pragma once
#ifndef THREADPOOLX_H
#define THREADPOOLX_H

/* Lessons learned:
 You can use initializer list on constructor in .cpp, too. In this
 way, you don't use initilizer list on the constructor in .hpp

 In thread pool, there are 2 pools. Pool of threads. Pool of tasks.
 Pool of threads picks up the tasks from the pool of tasks, and executes
 the tasks.

 You can call assert of locks??

 decltype(function()).... <== need to actually CALL the function.
 (i.e. use parenthesis after "function")

 Wheneve a task is queued by a user, the queueTask() needs to notify
 one thread that's been waiting for the user to queue a task.

*/

#include <limits>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <map>
#include <queue>
#include <future>
#include "orderedRunnableX.hpp"
#include "pooledTaskX.h"

//const static NONPOOL = std::numeric_limits<int>::max();
//const static 

class ThreadPoolX{
private:
	typedef std::pair<unsigned, int> OrderType;
	typedef OrderedRunnableX<OrderType> PrioritizedRunnable;

	int m_priority;
	std::condition_variable m_cv;
	std::mutex m_mtx;
	std::map<std::thread::id, std::thread> m_poolOfThreads;
	std::priority_queue<PrioritizedRunnable> m_poolOfTasks;
	unsigned int m_threadID;
	unsigned int m_numThreads;

	enum Priority{
		INPOOL = 0, INPROGRESS = 1, DONE = 2
	};


public:

	ThreadPoolX(int numThreads = 1)
		: m_threadID(0), 
		  m_numThreads(0){	//TODO: instead of keeping m_numThreads, use the size of the pool of threads... but this means the pool of threads has some usefulness, which I don't know right now.
		setNumThreads(numThreads);
	}

	template<typename Any>
	auto queueTask(Any&& func) -> std::future<decltype(func())>{
		typedef decltype(func()) ReturnType;
		//TODO: try this syntax ==>  std::unique_ptr<PooledTaskX<ReturnType> > task(func);	
		std::unique_ptr<PooledTaskX<ReturnType> > task;
		task.reset(new PooledTaskX<ReturnType>(func));
		std::future<ReturnType> result = task->getResult();
		
		int priority = INPOOL; 
		std::unique_lock<std::mutex> lck(m_mtx);
		unsigned int threadID = ++m_threadID;
		OrderType order = std::make_pair(threadID, priority);	// in Lindley's code they are switched. But I don't think you have to follow "<unsigned, int> pattern"
		m_poolOfTasks.emplace(PrioritizedRunnable(std::move(task), order));	
		
		// Need to manually notify condition variable. Otherwise, the cv.wait() in prcoessing task part of a thread will hang forever.
		m_cv.notify_one();
		return result;		// return future, instead of future.get(). Future.get() is a blocking call.
	}

private:
	void setNumThreads(int numThreads);
	void threadMainLoop();
	PrioritizedRunnable getWaitingTask();


};


#endif
