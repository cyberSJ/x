#include "threadPoolX.h"

// Increase the number of registered threads upto the limit specified
// by the user.
void ThreadPoolX::setNumThreads(int numThreads){
	// Routine for adding threads
	while (m_numThreads < numThreads){
		std::thread th(&ThreadPoolX::threadMainLoop, this);
		//m_poolOfThreads.emplace(th.get_id(), std::move(th));
		th.detach();
		m_numThreads++;
	}

	// Routine for deleting threads.
	//while (m_numThreads > numThreads){
	//	std::unique_lock<std::mutex> lck(m_mtx);
	//	m_cv.wait(lck, []{!return m_poolOfThreads.empty();});

	//	
	//}
}

void ThreadPoolX::threadMainLoop(){ 
	// TODO: how do you receive PR?
	while(1){
		PrioritizedRunnable pr = getWaitingTask();
		if (pr){
			pr();
		}
	}
}

ThreadPoolX::PrioritizedRunnable ThreadPoolX::getWaitingTask(){
	std::unique_lock<std::mutex> lck(m_mtx);
	m_cv.wait(lck, [this]{return !m_poolOfTasks.empty();});

	PrioritizedRunnable pr = std::move(m_poolOfTasks.top());
	m_poolOfTasks.pop();
	return pr;		// NRVO can return object w/o unnecessary copies.
}	// going out of scope dis-owns the lock.
