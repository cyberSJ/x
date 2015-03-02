#ifndef dsHashSC_H
#define dsHashSC_H
#include <vector>
#include <list>
#include <algorithm>
#include <string>

template<typename HashedObj>
class dsHashSC{

	public:
		/*
			[step1] Constructor
			Just initialize the m_currentSize.
			There's no need allocate space for the vector of lists
			because it will done dynamically and automatically by
			the std::vector class.

			Actually, no. Since we will be accessing the vector
			using hash value, the vector with the m_currentSize
			must be created.
		*/
		explicit dsHashSC(int size = 101):m_currentSize(size){	
			m_lists.resize(m_currentSize);
		}

		/*
			[step2] Iterating Mechanism
			Decribe how we should travel between each building block.
			We don't need to do anything because our building blocks,
			vector and list, already have good iterating mechanism.
			
			However, the iterating mechanism invovles hashing because
			hashing determines where to refer to.
		*/



		/*
			[step3] Accetor functions
			Functions that doesn't modify the structure/content of the
			data structure. These should be something like this:
			contains, find, etc.
		*/	
		/*
			[step3.1] contains.
			Accept an HashedObj and determine if it exists in the DS.
			returns bool indicating success.
		*/	
		bool contains(const HashedObj &x) const{
			int hashVal = myHash(x);
			const std::list<HashedObj> &currentList = m_lists[hashVal];
			return (std::find(currentList.begin(), currentList.end(), x) != currentList.end());
		}

		/*
			[step4] Modifier funciton
		*/
		
		/*
			[step4.1] insert.
			Accept an HashedObj to insert. Compute hashvalue, and then
			push to the front of the list because inserted elements
			are more likely to be accessed again in near future (really??)
			Do nothing if there's already the element 
			Returns bool for success.
		*/
		bool insert(const HashedObj &x){
			int hashVal = myHash(x);
			std::list<HashedObj> &currentList = m_lists[hashVal];
			if (std::find(currentList.begin(), currentList.end(), x) != currentList.end()){
				// item is already present
				return false;
			}
			currentList.push_back(x);

			if (++m_currentSize > m_lists.size()){
				//rehash();
			}

			return true;
		}
		
		/*
			[step4.2] remove.
			Accept the HashedObj to remove.
			Comptue the hash value as usual, see if the element already exist.
			If not return false, if exists, remove it from the list, and 
			return true. Decrement the m_currentSize
		*/	
		bool remove(const HashedObj &x){
			std::list<HashedObj> &currentList = m_lists[myHash(x)];
			typename std::list<HashedObj>::iterator iter = std::find(currentList.begin(), currentList.end(), x);	// Member variable or Nested type?

			if (iter == currentList.end()){
				return false;
			}

			currentList.erase(iter);
			--m_currentSize;
			return true;
		}
		
		

	

	private:
		/*
			[step0] Building block of data structure.
			Instead of creating our custom class for the building block,
			we use already exisitng vector and list.
		*/
		std::vector<std::list<HashedObj> > m_lists;
		int m_currentSize;
	
		/*
			[step2.1] myHash
		*/		
		int myHash(const HashedObj &x) const{
			int hashVal = dsHash(x);
		
			hashVal %= m_lists.size();	// get number between -listSize ~ +listSize
			if (hashVal < 0)
				hashVal += m_lists.size();	// get number tween 0 ~ +listSize
			
			return hashVal;
		}


};



/*
	[step5] Global functions
*/

/*
	hash
	Refer to pg.188 of the Data Structure and Algorithm Analysis in C++
	by Mark Allen Weiss.
*/
int dsHash(const std::string &key){
	int hashVal = 0;
	
	for (int i = 0; i < key.length(); i++){
		hashVal = 37*hashVal + key[i];
	}

	return hashVal;
}

class Employee{
	public:
		Employee(std::string &name, double salary = 0, int seniority = 0) : m_name(name), m_salary(salary), m_seniority(seniority){

		}

		
		const std::string & getName() const{
			return m_name;
			//return &m_name;
		}	

		/* 
			operator== is used by std::find() and somewhere inside 
			std::find()
			there is an accessor (const signature) that uses this
			operator==() so this operator==() must also have
			const signature.
			
		*/	
		bool operator==(const Employee &rhs) const{
			return getName() == rhs.getName();
		}	
	
		bool operator!=(const Employee &rhs){
			return getName() != rhs.getName();
			// return !(*this == rhs);
		}

		

	private:
		std::string m_name;
		double m_salary;
		int m_seniority;

};

int dsHash(const Employee &emp){
	return dsHash(emp.getName());
}




#endif
