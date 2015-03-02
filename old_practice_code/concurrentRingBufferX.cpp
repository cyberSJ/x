
/* ConcurrentRingBuffer

 This is a wrapper of boost::circular_buffer.
 This class should contain any type (i.e. it must be a template)
 Any public methods first acquires the internal mutex member variable.

 Basic simple functions:
 clear: clears the circular buffer
 size: get the size of the circular buffer
 empty: tell if circular buffer is empty
 full: tell if circular buffer is full
 reserve: tell how many element we can write immediately to the buffer
          (use reserve())
 ALWAYS LOCK THE MUTEX before calling public member functions.

 Pushing to the concurrent buffer:
 -----------------------------------
 Wait on the lock until the concurrent_buffer is NOT full.
 (i.e. Check concurrent_buffer is not full, and when it isn't,
 obtain lock only when other thread notify_all() or notify_one().)
 When the wait is over, push_back the data (type T) to the
 circular buffer. Release the obtained lock, and notify_one()
 using the condition variable (which was one of the member variables).
 Also, implement another function which pushes multiple elements
 using iterators(start iterator and end iterator).

 Poping from the concurrent buffer:
 ------------------------------------
 Return through argument AND return by reference... why? idk...
 Wait until circular buffer is not empty AND other thread notify_one(),
 then acquire the lock, pop the front of the buffer. Release the lock
 and use member condition_variable to notify_one() other thread.

 Faster concurrentRingBuffer:
 ------------------------------
 Assumes 1 producer & 1 consumer, which means we don't need mutex.
 Also use some public-domain CircularFifo class instead of 
 boost::circular_buffer.
 TODO: Visit the website Lindley mentioned when using 1P1C version.

*/
