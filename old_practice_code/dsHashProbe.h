#ifndef dsHashProbe_H
#define dsHashProbe_H

#include <map>
#include <vector>
#include <iostream>
#include "dsHashSC.h"
#include <math.h>

template<typename HashedObj>
class dsHashProbe{

	public:
		/*
			[step1.1] Building block - lazy deletion.
		*/
		enum EntryType {ACTIVE, EMPTY, DELETED};

		/*
			[step2] Constructor 
			Accept the initial size, and then make all
			entries to EMPTY.
		*/
		// TODO: Try this block instead of below.
		//dsHashProbe(int size = 101):m_currentSize(size){
		//	for (int i = 0; i < m_currentSize; i++){
		//		m_array[i].info = EMPTY;


		//	}
		//	
		//	// TODO: Try this instead of for loop:
		//	// vector::resize
		//	// std::fill
		//	
		//}

		dsHashProbe(int size = 101):m_array(nextPrime(size)){	// TODO: is this how to initialize an array??:
			makeEmpty();	// Use makeEmpty() because this can be a public function invoked separately from constructor
		}

		/*
			[step2.1] makeEmpty
			Set the m_currentSize to zero.
			So m_currentSize means the number of actual
			variables stored in the hashtable.
			Then, make all the slot status to be EMPTY.
		*/
		void makeEmpty(){
			m_currentSize = 0;
			for (int i = 0; i < m_array.size(); i++){
				m_array[i].info = EMPTY;
			}
		}

		/*
			[step3] Accessor functions.
			Contains
		*/
		bool contains(const HashedObj & x){
			// Calculate the hash function, and find the closest
			// available slot using quadradic probing.
			// See if the slot is Active
			int currentPos = findPos(x);
			return isActive(currentPos);
		}

		int findPos(const HashedObj & x) const{
			int offset = 1;
			int currentPos = myHash(x);

			while (m_array[currentPos].info != EMPTY &&
				m_array[currentPos].s_element != x){
				currentPos += offset;
				offset += 2;
			
				if (currentPos >= m_array.size()){
					currentPos -= m_array.size();		
				}
			}

			return currentPos;
		}

		bool isActive(int position){
			return m_array[position].info == ACTIVE;
		}

		/*
			[step4] Mutator functions.
			Insert and remove.

			insert: Accept an HashedObj, calculate the position,
			and then copy the object into the slot,
			and set the info the active.
			Return bool for successfulness.

			remove: Accept an HashedObj to remove.
			Find position using hash functions.
			Check if the slot is Active and it is indeed the element
			we want to lazy-remove.
			Then lazy-remove, but don't reduce the m_currentSize.
			(because the element is still there.)
			Return bool for successfulness.
		*/
		bool insert(const HashedObj &x){
			int currentPos = findPos(x);
			if (isActive(currentPos)){
				return false;
			}

			// If here, item does not exist in hashtable.
			m_array[currentPos] = HashEntry(x, ACTIVE);

			// Rehash
			//if ( ++m_currentSize > m_array.size()/2 ){
			//	rehash();
			//}

			return true;
		}

		bool remove(const HashedObj &x){
			int currentPos = findPos(x);
			
			if(!isActive(currentPos)){
				return false;
			}

			// Lazy deletion
			m_array[currentPos].info = DELETED;
			return true;
		}

		

	private:
		/*
			[step1] building block of data structure.
			Since we are doing lazy deletion,
			we need another variable other than the
			actual data to mark the slot as active,
			empty, or deleted.
		*/
		struct HashEntry{
			HashedObj s_element;
			EntryType info;

			HashEntry(const HashedObj & e = HashedObj(), EntryType i = EMPTY)
				: s_element(e), info(i){
			}
		};

		
		/*
			[step1.2] Member variables
		*/
		std::vector<HashEntry> m_array;
		int m_currentSize;

		/*
			[step2.5] Iterating Mechanism.
			Hashing function.
		*/
		int myHash(const HashedObj &x) const{
			// Get the hash of x
			int hashVal = dsHash(x);

			// Make it in range of our hashtable size
			hashVal %= m_array.size();

			// Take care of negative hash value.
			if (hashVal < 0){
				hashVal += m_array.size();
			}
	
			return hashVal;
		}
	
		/*
			[step5] rehash
		*/
		void rehash(){
			std::vector<HashEntry> oldArray = m_array;

			// Empty the m_array while doubling the size
			m_array.resize(nextPrime(2 * m_array.size()));
			for (int i = 0; i < m_array.size(); i++){
				m_array[i].info = EMPTY;
			}

			// Dump back 
			m_currentSize = 0;
			for (int i = 0; i < oldArray.size(); i++){
				if (oldArray[i].info == ACTIVE){
					insert(oldArray[i].element);
				}
			}
		}

		/*
			http://www.programmingsimplified.com/c/source-code/c-program-find-next-prime-palindrome
		*/	
		int nextPrime(int number){
			long t, d, c;
			long r = 0;
			while (true){
				number++;
				t = number;

				while(t){
					r *= 10;
					r += t%10;
					t /= 10;
				}
				
				if (r == number){
					d = (int) sqrt(number);

					for (c = 2; c <= d; c++){
						if (number%c == 0) break;
					}
					if ( c == d+1 ) break;
				}
				r = 0;

			}

			return number;
		}



};

#endif
