// Practiciing how to create a thread pool.

	Thread pool is a class that manages multiple threads.
	An object may have a thread pool as its member.

	Each thread has certain state: DETACHED, JOINABLE, COMPLETE.
	A task is assigned a thread. You need to create this task
	object.

	Runnable class:
	---------------------
	This task object is an inheritance of runnable.
	A runnable is an operator()-overloaded class.
	When you call someClassInstance(), the class "runs", which is
	why the class is called runnable. Task class is a runnable.
	
	Pooled Task class:
	----------------------
	Task is a class known by the thread pool class.
	Task class has std::packaged_task that returns an arbitrary 
	template type, but takes no argument (because of "runnable").
	Since it has a packaged_task as its class member, Task class
	must contain a function to return the future of the packaged_task
	Since Task is a runnable, it also must have operator() overloaded.
	In that overload, packaged task is called with operator().
	(packaged_task should be initialized with the arbitrary
	class).
	The constructor of the PooledTask should accept rvalue reference
	of the function the packaged_task will execute. It is rvalue 
	reference because we want to, in the future, MOVE the function
	to be thread (and not copy the function) since the function's
	internal have precious things that we don't want to copy, but move.

	Now, back to thread pool class...
	The class also contains thread_local that contains priority value.
	It has 3 kinds of priorities: NONPOOL, IDLE, other.
	NONPOOL = basically says this thread is not in the thread pool.
			  (HIGHEST VALUE)
	IDLE = this thread just got added to the thread pool
			(LOWEST VALUE)
	other = priority can be updated from the return value of one of
			the functions in the runnable class.
	
	std::condition_variable is used. TODO: what does it do exactly?

	OrderedRunnable class:
	-------------------------
	It contains unique pointer to the Runnable instance AND order
	of arbitrary type. Basically, this is a class for mechanisms for
	those two member variables.
	OrderedRunnable will be contain std::pair in thread pool.
	When it does, it becomes, PrioritizedRunnable, which is a 
	basic building block (member variable) of the thread pool.
	OrderedRunnable provides interface for comparing priority,
	and operator bool() overloading, and rvalue move constructor,
	rvalue assign constructor, operator() overloading, and 
	getter for the order.

	Thread pool also contains std::mutex.

	It also keeps a std::map of thread id and thread itself.

	it keeps thread id as an integer.



	Step-by-step inst. of ThreadPool class creation:
	====================================================

	Creation of ThreadPool class:
	-----------------------------------
	When thread pool is created, it sets a fixed number of threads.
	Keep track of the created threads by making a pair of threadID and
	thread itself. When creating a thread, use the main thread loop as
	the function to implement in a thread.
	
	Advanced thread pool creation has more steps to
	make sure the number of created threads in the pool is really what
	the user has requested.

	The main thread loop has while loop that loops until a task is
	processed (i.e. a task has finished processing). Then it actually
	removes the thread from the pool... 
	(I don't know if one should reomve it..)
	The pool is std::map of threadID and thread.


	Processing a task:
	--------------------------
	Once the thread main loop is at the stage where it waits for
	a task to be finished, the task needs to be processed.

	First, it obtains the waiting task.


	Getting a waiting task:
	-------------------------
	This function waits for a condition_variable and also waits
	if task queue is empty. Task queue is a std::priority_queue 
	of prioritized runnable. Prioritized runnable is an ordered 
	runnable with std::pair<unsigned, int> as a template type.

	Then it moves the top of the task queue and returns it. But before
	it returns, we need to notify one other thread waiting for
	the class member condition_variable.


	Back to processing a task:
	-------------------------------
	After getting the waiting task,
	since prioritized runnable has operator bool() overloaded,
	we can utilize that to first see if the task is valid (exists).
	Then if it exists, we can run the task. But before we run the
	task, there are some bookkeeping for thread pool.

	This bookkeeping is temporarily updating the priority of the 
	"order" of the runnable (remember that runnable had an "order"?).
	After the runnable is ran, we notify_all....TODO: notify_one

	Now we build the user interface to the thread pool.


	Register a task into the thread pool:
	---------------------------------------
	User would like to push a function to the thread pool.
	This is done by queueTask().

	queueTask() should accept a rvalue reference of the function since
	we want to move the task and not copy (copy is expensive and
	we don't want to copy the functions internal data when it is not
	supposed to be copied).

	User will also want to know the result of the input function 
	after the thread pool finishes executing the function.
	(This is why pooledTask is a wrapper of packaged_task)
	We need to return the future of the pooledTask.
	However, future also needs the return type of the input function.
	Since thread pool do not know the return type of the input function,
	we use auto, ->, and decltype() as signature of the queueTask.

	Since our PrioritizedRunnable accepts order and unique pointer
	to a runnable function, we need to create an order and a unique 
	pointer	to the input function.

	Order is a std::pair of priority and threadID.
	Priority is 1, and threadID is the next higher integer in
	the thread pool class (starting 0 I guess...).

	Then, we finally place the input function and the order to the
	std::priority_queue (thread pool class member variable).
	
	The unique pointer needs the type of the pointer. This is the
	pooledTask (inheritance of Runnable). But the pooledTask
	requires the return type of the function. This is where we use
	decltype of the accepted function.

	We also assign id to our new thread 

	Then we construct PrioritizedRunnable and push it to the 
	thread pool. The thread pool uses std::priority_queue to
	contain multipled PrioritizedRunnables. Priority queue is
	more like heap than queue. It is ordered in priority.

	Finally, we notify one other thread for continued processing.
	But all these process requires that other threads don't
	change the contents of the thread pool, (since we
	are creating one more threads in the pool) so we need std::mutex,
	which is the data member of thread pool.

	Lindley implemented a scopedLock which is a wrapper around
	the lock, but I think I can just use unique lock, which
	unlocks the mutex automatically when the lock goes out of the
	scope.

	Really finally, we then return the std::future of the 
	pooledTask of the input function.

	Constructor (again):
	-------------------------
	Creates a thread pool of size 9 threads.. I don't know why 9..
	But Lindley's code never creates a thread pool of size 0 threads.
	I don't know why...

	First, you create a new thread. Lindley simplified this routine
	by making another routine called threadMainLoop and binding
	the threadMainLoop routine with "this" object and calling
	thread with that binding.
	
	You also get the thread id from the created thread. Thread
	object can return thread id through API.

	Then you need to detach from the thread so that our constructor
	can do remainder of the bookkeeping of threads.

	Then we need to put our thread into the thread map so that
	we can keep track of the threads within this thread pool.
	The thread map accepts thread id and the thread instance.
	Make sure you don't copy but move the thread instance to the
	map. Copying thread instance is obviously bad.

	We keep creating/pushing thread into the thread map until
	we have all required number of threads created.

	We then need to account for the case when the number of threads
	pushed into the thread map is larger than that requested.
	This can happen when someone calls the setNumThreads routine
	elsewhere other than in constructor. (So setNumThreads is a
	subroutine used in ctor to register threads into the thread pool)

	Removing a thread is not like manually removing a thread.
	You have REQUEST a thread to be removed. You do this by INSERTING
	a stopThreadTask, which is a Prioritized runnable with idle
	priority, invalide task ide, and NULL (null pointer to a thread),
	into the thread pool.

	You REQUEST a thread to be reomved because you don't kill a thread
	yourself, but you want to WAIT until a thread is finished.

	Pushing a stopThreadTask is like zero-padding. The number of 
	stopThreadTasks you push is equal to the amount of #{active 
	threads} - #{threads user requested}. Once you added correct
	amount of stopThreadTasks, you wait on the condition variable
	and wait until #{active threads} == #{threads user requested}.

	Creating new thread and pushing stopThreadTask requires mutex
	locked because... idk

	Destructor:
	-------------------
	The principle is, you request the thread pool to have 0 size.
	Set the number of threads in the thread pool to 0
	You obtain mutex, then wait on the confition variable until
	the size of the thread pool becomes 0.

	threadMainLoop (again):
	----------------------------
	In this function, our goal is to process the job given to this
	thread, and notify another thread that our job is done.
	
	First obtaining lock is important...idk

	In order to process the job given, we need to get the task
	that is at the top of the priority queue (which is the thread pool).
	
	In order to get the task, we wait on the condition variable
	until the thread pool has some task in it.

	When the thread pool has a task, then we get the task from
	the priority queue. In order to reflect any change we do
	to the task, we don't get a copy of the task, we obtain the
	task itself by using std::move().

	Then we delete the emptied task (due to std::move()) in the
	thread pool.

	If there are any tasks even if we got the top task in the
	priority queue, then we notify other thread to process that
	other tasks.

	Then we return the task. In this return, we can return by copy
	if we are just using the content of the thread, and the 
	changed content does not need to be reflected to any other 
	process. We've already took out the task from the thread pool,
	so the thread pool doesn't need to know anything at all what
	we do to the taken out thread.

	After obtaining the task, we check for task existance, and
	if it exist, we update the current thread priority temporarily
	before calling the runnable task. We do priority updating
	because we want our current thread to be not idle. Before updating,
	our thread was set idle, but we temporarily set non-idle,
	so that our thread has non-idle priority. I don't know who
	cares about our thread's priority at this point, but Lindley's code
	does this at this point.

	After the priority update, we notify all other thread through
	the condition variable. This is done because maybe other
	threads are waiting for for thread pool to be empty or something?
	TODO: need to find out why we notify all.

	After processing is done, we erase current thread from the
	current map based on thread id. 
	TODO: I don't know why we keep both priority queue and map.
	
	Then we notify one.... TODO: Why notify one at this stage?


	How to request stopping a thread that is detached??:
	-------------------------------------------------------
	I think you can't. But in thread pool, you don't request
	a detached thread to be stopped, but you just insert a task 
	(another prioritized runnable) into the pool of tasks, that
	basically has order information and null unique pointer an empty
	task would have. This signals that a task is to be removed from
	the pool of tasks.
