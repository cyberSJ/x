#pragma once
#ifndef POOLEDTASKX_H
#define POOLEDTASKX_H

/*	Lessons Learned:
 Pure abstract class doesn't have to have constructor.
 How to call std::packaged_task with no input argument function.
 Why we have to MOVE (instead of copy) the function that will be 
 wrapped around the std::packaged_task.

*/

#include <thread>
#include <future>

template<typename returnT>
class PooledTaskX : public RunnableX{
// protected in original version... idk why.
protected:
	std::packaged_task<returnT (void)> m_task;

public:
    // Need a template with another typename than returnT.
	// (the typename for the return type of the taskFunction.
	//PooledTaskX(returnT taskFunction)	// wrong..
	template <typename Any>
	PooledTaskX(Any&& taskFunction)
		: m_task(taskFunction)
	{}

	// The result of the pooled task can be retrieved here.
	std::future<returnT> getResult(){
		return m_task.get_future();
	}

	// Runnable call
	void operator()(){
		//returnT();	//wrong.. can't call type by itself.
        m_task();	// correct.. this is how you call empty-argument
		 			// std::packaged_task
	}

};


#endif
