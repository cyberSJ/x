#ifndef dsList_H
#define dsList_H
#include <iostream>

template<typename Object>
class dsList{
	private:
		/*
			Node struct.
			Has data of type Object, previous node pointer,
			and the next node pointer. It also has a constructor
			All members public by default, i.e. usuable by dsList
			class.
		*/
		struct Node{
			Object m_data;		// why the data should not be a pointer? Let's try using pointer instead.
			Node *m_prev;
			Node *m_next;

			/*
				Constructor.
				Accepts object (constructo one by default),
				previous node (default to null), and next node (default to null).
				Initialize these stuff with the input arguments.
				What does it mean to construct an object for default argument?
				--> create a node that has a data content in it.
			*/
			Node(const Object &d = Object(), Node *p = NULL, Node *n = NULL)
				: m_data(d), m_prev(p), m_next(n){
	
			}
		};

	public:
		/*
			const_iterator class.
			In dsVector, the iterators were just pointer to an Object, but here iterators are class because iterator not only points to data, but points to a Node struc, so more functions are needed.
			DONE: Why do we return-by-value for iterators for functions such as insert()?: because we construct a new iterator locally inside the function insert().
		*/
		class const_iterator{
			public:
				/*
					Constructor.
					Zero-parameter, initializes current node pointer to null
				*/
				const_iterator() : m_current(NULL){

				}

				/*
					operator*.
					Accessor function that returns the data of current node.
					Uses retrieve(). DONE: why?: not much reason, just for clean code.
					First const is because we are using const_iterator where the refereced, iterated element is not supposed to be changed.
					Second const is because this is an accessor function.
				*/
				const Object & operator*() const{
					//std::cout <<"I'm in const_iterator's accesor version of operator*()" << std::endl;
					return retrieve();
				}

				/*
					prefix operator++.
					Go to the next node, return updated address of the const_iterator.
					Returns Node & because what we are returning is the memory address of the node. <== Wrong... returns const_iterator &...DONE: why is this wrong?: because operator++ is not interested in changing the content, it just moves the iterator. (i.e. it just points to different content.
				*/
				//const Node & operator++() const
				const_iterator operator++(){		// DONE: try to put const keyword in front of the const_iterator: Nothing happens when iterating using for loop.
					m_current = m_current->m_next;
					//return &m_current;		// DONE: try this with the above todo. This is returning just an address of the m_current pointer: It doesn't work since this line returns reference of Node pointer, whereas the function should return reference of const_iterator.
					//std::cout << "inside prefix operator++. this = " << **this <<  std::endl;
					return *this;		// Here, we are returning the entire const_iterator class (with the modified m_current)
					//DONE: can we return void and functionally stay the same?: YES in for loop iteration. But iter = ++iter doesn't work because we are returning void.
				}
				

				/*
					postfix operator++
					Take fake int parameter. Return entire class address. 
					Save a copy of this const_iterator object, increment this object, but return the copy. This way, we are returning an incremented value but also current updated one. It's like returning a skewed content. 
					DONE: Why are we not returning reference but an actual const_iterator object?: because we need to return old, which is created locally.
				*/
				const_iterator operator++(int){
					const_iterator old = *this;
					//std::cout << "before changing this. this = " << **this << "    old = " << *old << std::endl;
					++(*this);		// Why *this and not this? because this is a pointer to const_iterator object, but the operator++ is overloaded for the const_iterator object (not for the pointer to the const_iterator object)
					//std::cout << "after changing this. this = " << **this <<  "    old = " << *old << std::endl;
					return old;		// DONE: how does compiler know how to return-by-value const_iterator, which is a custom class?: It does shallow copy. But this and old are two separate entities. old's node pointer still has the address of [m_current], but because of ++(*this), this's node pointer nod has the address of [m_current->m_next] (and doesn't affect what old's node pointer has.
				}


				/*
					operator--
					Accepts nothing, move the pointer to the previous node and return the iterator	
				*/
				const_iterator operator--(){
					m_current = m_current->m_prev;
					return *this;
				}
				const_iterator operator--(int){
					const_iterator old = *this;
					--(*this);
					return old;
				}
	




				/*
					operator==
					Returns true if the const_iterators pointing to the same node. <-- DONE: is this wrong?: Then operator== will return true even if there were two different Node pointers referting to the same node. Let's say if different nodes were somehow (erronicallY) pointing to a same object, then the iterator will return true, and those two different nodes will be concluded to be the same. This is not what we want.
					Returns true if the const_iterators' node pointers are really the same iterators.
					Accepts another const_iterator object.
				*/
				bool operator==(const const_iterator &rhs) const{
					return m_current == rhs.m_current;	// DONE: what happens when we compare contents of each other?: it won't work, since two node's might have same Object and it would not be correct. 
				}

				/*
					operator!=
					Returns trus if two const_iterators are not the same iterator
					Accepts another const_iterator object, and compares the member variable of it with itself.
					Utilizes operator==
				*/
				bool operator!=(const const_iterator &rhs) const{
					return !(*this == rhs);
				}
				

			protected:
				Node *m_current;
				const dsList<Object> *m_dsList;

				/*
					retrieve.
					get the data from the iterator.
				*/
				Object & retrieve() const{	// DONE: why the const keyword should be here?
					return m_current->m_data;
				}

				/*
					Another constructor.
					This constructor accepts a Node information and initializes itself with with Node.
				*/
				const_iterator(const dsList<Object> &lst, Node *p) : m_current(p), m_dsList(&lst){//TODO: Why do we accept the dsList referece and not a pointer?We should try accepting a pointer.
				}

				friend class dsList<Object>;

				void assertIsValid() {
					if (m_dsList == NULL || m_current == NULL || m_current == m_dsList->m_head){
						throw IteratorOutOfBoundsException();
					}
				}


				int IteratorOutOfBoundsException(){
					std::cout << "Error: Not a valid iterator!!" << std::endl;
					return 1;
				}
				
		};
		
		/*
			iterator class.
			isterator IS-A const_iterator
		*/
		class iterator : public const_iterator{
			public:
				/*
					Zero-parameter constructor.
					Why do I need to do nothing?: because it inherits member variables from the parent class.
				*/
				iterator(){
				}

				/*
					operator* mutators and accessors.
					It returns the contents of the node that it pointed to. (i.e. the iterator points to node, and node's content, which is type object is returned.)
					Uses parent class's retrieve function to do the job. 
					The only changes are in the what return type the function returns with the same return value.
					TODO: Unresolved. why mutator version of iterator class hides the accessor version of the parent class?: it's just what C++ is. But I don't know when the dsListMain.cpp the accessor version of iterator::operator*(). I'm not able to use it at all. I always end up using the mutator version.
				*/
				Object & operator*(){
					//std::cout <<"I'm in iterator's mutator version of operator*()" << std::endl;
					return const_iterator::retrieve();
				}
				const Object & operator*() const{
					//std::cout <<"I'm in iterator's accesor version of operator*()" << std::endl;
					//const_iterator::retrieve();	// DONE:let's try this line: It works. It's just that using const_iterator::operator*() is doing things in higher level.
					return const_iterator::operator*();
				}

				/*
					operator++
					Why do we need to overload this operator++ again?: because operator++ should return an iterator by default, and we need iterator (and not const_iterator) to be returned.
					Accepts no argument, increment iterator position, returns iterator reference using this pointer.
				*/
				iterator & operator++(){
					const_iterator::m_current = const_iterator::m_current->m_next;
					//const_iterator::operator++();		// DONE: we should try this instead: It works, and this should be done if you consider using higher-level option.
					return *this;
				}
				iterator operator++(int){
					iterator old = *this;
					//const_iterator::operator++(1);	// DONE: we should try this instead: It works!!!
					++(*this);
					return old;
				}

				/*
					operator--
					Accepts nothing, move the pointer to the previous node and return the iterator	
				*/
				iterator operator--(){
					const_iterator::m_current = const_iterator::m_current->m_prev;
					return *this;
				}
				iterator operator--(int){
					iterator old = *this;
					--(*this);
					return old;
				}

			protected:
				/*
					One-parameter constructor
					Used by dsList class.
					Accepts a node pointer, and initialize iterator class's m_current with that pointer. However, in order to do that, utilize the constructor of const_iterator since it is doing the same thing and const-ness does not affect this.
					Initializer list can be thought of as another command line.
				*/
				iterator(const dsList<Object> &lst, Node *p) : const_iterator(lst, p){
				}
		
				friend class dsList<Object>;
		};


		
	public:
		/*
			Constructor.
			uses init sequence.
		*/
		dsList(){
			init();
		}

		/*
			Destructor.
			Reclaims the member variables
			The clear sequence is used for: 
		*/
		~dsList(){
			clear();
			delete m_head;
			delete m_tail;
		}

		/*
			Copy constructor.
			Accepts another instance of dsList
			Copies the member varaibles. Copy the content of the pointer.
			DONE: copy constructor didn't work because *this = rhs <-- this line was doing a shallow copy, and when the original dsList was destructed, the destruction sequence for the copy at later time was trying to destroy what is already destroyed. The destructor of the copy-version was trying to assign some value to a destructed pointer, which is generates a segmentation fault. Solution is to provide a deep copy. CORRECTION: I was doing wrong when overloading operator=. The input argument was Object &, but should have been dsList &. This fault provided a shallow copy.
		*/
		//dsList(const dsList &rhs):m_size(rhs.m_size), *head(*rhs.head), *tail(*rhs.tail){	// why can't I do this?--> because this inefficient. This is inefficient because we should use operator= inside a copy constructor. Is the syntax wrong? Might be, because initializer list might now allocate a memory space for pointers. DONE 
		//dsList(const dsList &rhs) : m_size(rhs.m_size), m_head(rhs.m_head), m_tail(rhs.m_tail){		// this doesn't go well with the code we written...
		dsList(const dsList &rhs){
			init();		// Provide memory allocation and two-way linked list structure setup.
			*this = rhs;	// Takes advantage of the overloaded operators.
		}

		/*
			operator=

			Overall:
			-------------
			Self-checking,
			Basic concept: uses other functions to implement operator=().
			Clear the contents first. Then push in the rhs's content.	

			Details:
			-----------
			Self-checking utilizes operator==, instead of doing this != &rhs. What happens if we have used latter? DONE 
		*/
		const dsList & operator=(const dsList &rhs){
			if(this == &rhs){	// Let's try *this == rhs.... I don't know if it will work since we don't have operator==. DONE: yeah... we don't have proper operator==
				return *this;
			}
	
			clear();
			for (const_iterator iter = rhs.begin(); iter != rhs.end(); ++iter){
				push_back(*iter);
			}
	
			return *this;
		}

		/*
			begin
			Two versions: const and non-const
			Returns (const_)iterator that points to the beginning of the list. 
		*/
		iterator begin(){
			return iterator(*this, m_head->m_next);
		} 
		const_iterator begin() const{
			return const_iterator(*this, m_head->m_next);
		}
		
		/*
			end	
			Two versions: const and non-const
			Returns (const_)iterator that points to the location after the end of the list. 
		*/
		iterator end(){
			return iterator(*this, m_tail);
		}
		const_iterator end() const{
			return const_iterator(*this, m_tail);
		}

		/*
			insert
			Insert a Node infront of the input iterator.
			Accepts an iterator, and object to insert.
			Returns an iterator by value. DONE: why return iterator? maybe to chain???: Yes, since the iterator returned points to the newly added node, through which we can use to insert another Node.
		*/
		iterator insert(iterator iter, const Object & x){
			iter.assertIsValid();
			if(iter.m_dsList != this){
				throw IteratorMismatchException();	
			}
			Node *p = iter.m_current;	// Just to look nicer on the code
			m_size++;
			return iterator(*this, p->m_prev = p->m_prev->m_next = new Node(x, p->m_prev, p));
		}

		int IteratorMismatchException(){
			std::cout << "Error: iterator is not from the same dsList!!" << std::endl;
			return 1;
		}

		/*
			function: erase
			Two versions: erase at the iterator, or erase the entire list
			At iterator version: accept the iterator
			Return a new iterator that points to the Node after the current iterator
		*/
		iterator erase(iterator iter){
			//Node *n = *iter;		// content of iterator is a Node pointer. DONE: let's use this line instead below: Can't because content of iter is Object, whereas iter.m_current is Node pointer.
			Node *n = iter.m_current;
			//iterator retVal(*(++iter));	// DONE: try this line instead below. This won't work either because we are feeding Object to the constructor of iterator, which must accept a Node pointer.
			iterator retVal(*this, n->m_next);

			// Actual removing step
			n->m_prev->m_next = n->m_next;
			n->m_next->m_prev = n->m_prev;
			delete n;
			m_size--;

			return retVal;
		}

		iterator erase(iterator start, iterator end){
			// DONE: try this section of code: Works!!
			//for(iterator iter = start; iter != end; iter++){
			//	erase(iter);
			//}
			//return end;

			for(iterator iter = start; iter != end;){
				iter = erase(iter);
			}

			return end;
		}

		/*
			front, back, push_front, push_back, pop_front, pop_back
			front/back: accepts nothing returns the Object contained in the front/back of the list
			push_front/back: accepts an Object, push the Object to the front/back, returns nothing.
			pop_front/back: accepts nothing, erase the front/back Object, but don't return it. 
			
			Need to use erase(), iterators, maybe const_iterators, insert
		*/
		Object & front() {			// DONE: put const keyword after the parenthesis and see if it conflicts with the accessor version of front(): It doesn't work because puting const keyword after the parenthesis makes the function have same signature with the function below.
			return *begin();
		}
		const Object & front() const{
			return *begin();
		}
		Object & back(){
			return *--end();		// DONE: try *--end() and see if precedence is correctly understood: It works. The precedence is with operator--
		}
		const Object & back() const{
			return *--end();
		}

		void push_front(const Object & x){
			insert(begin(), x);
		}
		void push_back(const Object & x){
			insert(end(), x);
		}
		
		void pop_front(){
			erase(begin());
		}
		void pop_back(){
			erase(--end());
		}


		int size() const{
			return m_size;
		}
		bool empty() const{
			return size() == 0;
		}

		/*
			clear
			clears the list by going through the iterators and also set the size to 0.
			OR use begin() to 
			Should be implemented as the last stage of designing a class because clear() can use many other functions in the class to do the job.
			DONE: how can other function mimic the for loop with iterators?: with while loop checking if the list is empty and if not, keep removing one element at a time.
		*/
		void clear(){
			while(!empty()){
				pop_front();	//pop_front --> erase begin --> beginning node is after head node --> begnning node is erased.
			}
		}
	

	private:
		int m_size;
		Node *m_head;
		Node *m_tail;
		
		/*
			initialization 
			Allocates memory for member variable, initialize size, initialize head and tail to each other.
		*/
		void init(){
			m_size = 0;
			m_head = new Node;	// When do we actually use the constructor for the node? 
			m_tail = new Node;
			m_head->m_next = m_tail;
			m_tail->m_prev = m_head;
		}
	

};

#endif
