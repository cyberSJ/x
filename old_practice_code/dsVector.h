#ifndef dsVector_H
#define dsVector_H

/*
	Template for vector container.
	This is a template for any type of object
*/
template <typename Object>
class dsVector{

	public:
		/*
			Constructor.
			Takes the initial size as a variable (default = 0)
			Has initializer list for initial size and the capacity.
			The capacity if slight larger than initial size to avoid
			resizing at the beginning of constructor vector container.
			Then creates new primitive array of type object
		*/
		explicit dsVector(int initialSize = 0):m_size(initialSize), m_capacity(initialSize + SPARE_CAPACITY){
			m_objects = new Object[m_capacity];
		}

		/*
			Copy constructor.
			Takes another dsVector and initialize the array of type object to null.
			Then uses operator= to do the job of copying.
			m_objects(NULL) is required because the operator= assumes that there is a pre-existing m_objects.
			We don't need the return value of operator= in copy constructor. The reason for the return value is for when we chain the operators like a=b=c;
		*/
		dsVector(const dsVector & rhs):m_objects(NULL){
			//operator=(rhs);
			*this = rhs;
		}

		/*
			Destructor.
			Just reclaim the new-ed member variable.
		*/
		~dsVector(){
			delete [] m_objects;
		}

		/*
			operator=.
			Returns const reference because	we want to chain multiple ='s
			(since operator= accepts const reference). operator='s scope is
			lhs=rhs. It's just that this entire statement "lhs=rhs" returns
			a const reference (and not that "=rhs" returns const reference)
			First checks for identity, clear out the memory for primitive 
			array before we do any operations on it, copy the size and the
			capacity, re-create the array of type object.
		*/
		const dsVector & operator=(const dsVector & rhs){
			if (this != &rhs){
				delete [] m_objects;
				m_size = rhs.size();
				m_capacity = rhs.capacity();
				m_objects = new Object[capacity()];	// calls this.capacity()
				for (int i = 0; i < size(); i++){
					m_objects[i] = rhs.m_objects[i];	// We should try rhs[i] later..because I think that should be the correct one.
				}
			}
			return *this;
		}

		/*
			resize.
			accepts new size. If the new size is larger than the capacity,
			reserve twice (plus 1) the new size...(why?: )
		*/
		void resize(int newSize){
			if (newSize > m_capacity){
				reserve(2 * newSize + 1);
			}
			m_size = newSize;
		}

		/*
			reserve.
			accepts new capacity. If trying to reserve smaller capacity
			than current vector size, just return.
			Otherwise, save the current address of the vector,
			assign new memory address for the new current vector,
			copy the old to new for each data item,
			copy the capacity,
			and reclaim the old vector memory space.
		*/
		void reserve(int newCapacity){
			if (newCapacity < size()){		// we should try size(), instead of m_size
				return;
			}

			Object *oldArray = m_objects;
			m_objects = new Object[newCapacity];
			for (int i = 0; i < m_size; i++){
				m_objects[i] = oldArray[i];
			}

			m_capacity = newCapacity;
			delete [] oldArray;
		}

		/*
			operator[] overloading.
			Two versions: accessor and mutator versions.
			Accepts index.
		*/
		Object & operator[](int index){		// Let's put const after the ()... since this function also doesn't change formal argument
			return m_objects[index];
		}
		const Object & operator[](int index) const{
			return m_objects[index];
		}

		/*
			host of commands.
			empty - check if size is zero
			size - return size
			capacity - return capacity.
		*/
		bool empty() const{	return m_size == 0;	}
		int size() const  {	return m_size;		}
		int capacity() const{	return m_capacity;	}

		/*
			push_back.
			accepts Object, reserve twice (plus 1) space if size has reached
			the capacity, then put the Object to the last space, and
			increment the size.
		*/
		void push_back(const Object & x){
			if (m_size == m_capacity){
				reserve( 2*m_capacity + 1);
			}
			m_objects[m_size++] = x;
		}

		/*
			pop_back().
			Doesn't return anything. Just conceptually decrease
			the size of the primitive array.
		*/
		void pop_back(){
			m_size--;
		}

		/*
			back.
			Returns the last object in the primitive array.
			Returns const reference to an object. Const because
			the usage of this back() already means that the user
			will not alter the value referenced as the return value.
			const after () because this back() function doesn't change
			any internal state of the dsVector class.
		*/
		const Object & back() const{
			return m_objects[m_size - 1];
		} 

		/*
			typdef'ing iterator.
			Must return two types of iterator: normal reference,
			and const reference. Const iterator are used when
			the internal state of the dsVector is supposed to not
			change when iterating.
		*/
		typedef Object * iterator;
		typedef const Object * const_iterator;

		/*
			begins and ends.
			Two versions for each type. Accessor and mutator versions.
			begin() returns the pointer to the beginning of the primitive array.
			end() returns the pointer to the end of the primitive array.
		*/
		iterator begin(){	// Let's add const after (), and let's change * to &
			return &m_objects[0];
		}
		const_iterator begin() const{
			return &m_objects[0];
		} 
		iterator end(){
			return &m_objects[size()];
		}
		const_iterator end() const{
			return &m_objects[size()];
		}

		/*
			Define SPARE_CAPACITY
		*/
		enum {SPARE_CAPACITY = 16}; 

	private:
		int m_size;
		int m_capacity;
		Object * m_objects;
};
#endif
