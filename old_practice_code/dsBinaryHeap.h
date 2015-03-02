#ifndef dsBinaryHeap_H
#define dsBinaryHeap_H

#include <vector>
#include <iostream>

template<typename Comparable>
class dsBinaryHeap{

	public:
		/*
			Constructor.
			Accept the initial m_currentSize, and 
			make the m_array that size.
		*/
		explicit dsBinaryHeap(int size = 100):m_array(size), m_currentSize(0){	// m_currentSize is the number element in the bineary heap (not the size of the m_array).
			//m_array.resize(m_currentSize);
		}

		/*
			Constructor 2.
			Accept vector of comparable.
			Then build a heap structure out of it.
			Use percolateUp repeatedly, but this is a naive method.
			Refere to pg.223 for smarter building heap
		*/
		explicit dsBinaryHeap(std::vector<Comparable> &array):m_array(array.size() + 10), m_currentSize(array.size()){
			//Construct a vector with heap-structure property only.
			for (int i = 0; i < array.size(); i++){
				m_array[i+1] = array[i];
			}
		}
		
		
		bool isEmpty() const{
			return m_currentSize == 0;
		}

		const Comparable & findMin() const{
			return m_array[1];
		}

		/*
			insert.
			Accept a comparable object.
			The initial position is the last index of the vector.
			(the last place of the heap) if the parent is larger, then push the parent in child's place. Then update the child's place to the parent.  Do this until there's no larger parent, or reach
			the top.
		*/
		void insert(const Comparable &x){
			if (m_currentSize == (int)m_array.size() - 1){
				//increase m_array's size.
				m_array.resize(2 * m_currentSize);
			}

			int hole = ++m_currentSize;	// because m_currentSize represent the number of nodes in the heap

			for (; hole > 1 && x < m_array[hole/2]; hole /= 2){
				m_array[hole] = m_array[hole/2];	// swap
			}
			
			m_array[hole] = x;
		}

		/*
			deleteMin
			Replace the root with the last element.
			Remove the old position of the last element.(?)
			Percolate down the new root.
			Return void
		*/
		void deleteMin(){
			if (isEmpty()){
				throw UnderflowException();
			}
			m_array[1] = m_array[m_currentSize--];
			percolateDown(1);
		}

		/*
			deleteMin and put it in the input.
		*/
		void deleteMin(Comparable & min){
			if (isEmpty()) throw UnderflowException();

			min = m_array[1];
			m_array[1] = m_array[m_currentSize--];
			percolateDown(1);
		}

		void makeEmpty(){
			m_array.clear();
			m_currentSize = m_array.size();
		}
		

	private:
		
		/*
			buildHeap
			Make the vector have a heap-order.
		*/	
		void buildHeap(){
			for (int i = m_currentSize/2; i > 0; i--){
				percolateDown(i);
			}
		}

		/*
			percolateDown. Bad way (3 comparisons per level)
			Accept an index of the Comparable object to percolate down.
			looks at left child, if left child is less, then swap.
			If not, then look at the right child, if right child is less,
			then swap. If not, terminate.
		*/
		//void percolateDown(int hole){
		//	Comparable tmp = m_array[hole];
		//	Comparable leftChild = hole * 2;

		//	if (leftChild != m_currentSize && m_array[leftChild] < m_array[hole])
		//		m_array[hole] = m_array[leftChild];
		//		m_array[leftChild] = tmp;
		//	else if(leftChild != m_currentSize && m_array[leftChild] > m_array[hole])
		//}
		
		/*
			percolateDown. Efficient way.
			(D + 1) comparisions, where D is the levels you get across in the heap.
			The bad way does 3D comparisions.
		*/
		void percolateDown(int hole){
			int child;
			Comparable tmp = m_array[hole];
			
			for (; hole * 2 <= m_currentSize; hole = child){
				child = hole * 2;
				if (child != m_currentSize && m_array[child + 1] < m_array[child])	// compare left and right child
					child++;		// select right child
				if (m_array[child] < tmp)//m_array[hole])
					m_array[hole] = m_array[child]; 	// swap with smaller child
				else
					break;		// found the spot!
			}
			m_array[hole] = tmp;
		}
		

		int UnderflowException(){
			std::cout << "ERROR: The BinaryHeap is empty!" << std::endl;
			return 1;
		}
		
		
		
		int m_currentSize;
		std::vector<Comparable> m_array;


};




#endif
