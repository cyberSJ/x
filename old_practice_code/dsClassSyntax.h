#ifndef DSCLASSSYNTAX_H
#define DSCLASSSYNTAX_H 

#include <vector>
#include <string>

class IntCell{
	public:
		IntCell();
		explicit IntCell(int initialValue);

		~IntCell();	// destructor
		IntCell(const IntCell &rhs);	// copy constructor
		const IntCell & operator=(const IntCell &rhs);

		//IntCell(int initialValue);
		int read() const;
		void write(int x);
		template <class Comparable> 
		const Comparable & findMax(const std::vector<Comparable> & arr){
			int maxIndex = 0;
			
			for (int i = 1; i < arr.size(); i++){
				if (arr[maxIndex] < arr[i]){
					maxIndex = i;
				}
			}

			return arr[maxIndex];
		}
	
	private:
		//mutable int m_storedValue;
		int m_storedValue;
};
		
#endif
