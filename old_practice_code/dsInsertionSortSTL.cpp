#include <iostream>
#include <functional>
#include <vector>


/***************************************************************************
	Function declaration. Important!!
	If you don't declare functions, you won't be able to compile the code
******************************************************************************/

template<typename Iterator>
void insertionSort(const Iterator &begin, const Iterator &end);

template<typename Iterator, typename Object>
void insertionSortHelp(const Iterator &begin, const Iterator &end, const Object &obj);

template<typename Iterator, typename Comparator>
void insertionSort(const Iterator &begin, const Iterator &end, Comparator lessThanFunctor);

template<typename Iterator, typename Comparator, typename Object>
void insertionSort(const Iterator &begin, const Iterator &end, Comparator lessThanFunctor, const Object &obj);




/*****************************************************************
	Primitive Two-parameter sort calls three-parameter sort
*******************************************************************/
template<typename Iterator>
void insertionSort(const Iterator &begin, const Iterator &end){
	insertionSortHelp(begin, end, *begin);		// begin is assumed to be used as an iterator, so we are assuming it can be dereferenced.

	//insertionSort(begin, end, std::less<*begin>());	// TODO: Let's try this
}

/*
	Three-parameter sort calls three-parameter sort.
*/
template<typename Iterator, typename Object>
void insertionSortHelp(const Iterator &begin, const Iterator &end, const Object &obj){
	insertionSort(begin, end, std::less<Object>());	// std::less needs an object, not an iterator
}


/*
	Primitive Three-parameter sort calling four-parameter sort.
	You can choose whatever comparator you want.
*/
template<typename Iterator, typename Comparator>
void insertionSort(const Iterator &begin, const Iterator &end, Comparator lessThanFunctor){
	// I think this if is quite unnecessary
	if (begin != end)
		insertionSort(begin, end, lessThanFunctor, *begin);
}

/*
	Primitive four-parameter sort
*/
template<typename Iterator, typename Comparator, typename Object>
void insertionSort(const Iterator &begin, const Iterator &end, Comparator lessThanFunctor, const Object &obj){
	
	Iterator j;

	for (Iterator p = begin+1; p != end; ++p){
		Object tmp = *p;	// save the moving variable
		for (j = p; (j != begin) & lessThanFunctor(tmp, *(j-1)); --j){	// I might place another object into the position originally occupied by tmp.
			*j = *(j-1);	// shift things to right. Remember that we are working with iterator, so we need the dereferencing.
		}
		*j = tmp;	// Store the moving variable.
	}
}



int main(int argc, char **argv){
	std::vector<int> unsorted;
	unsorted.push_back(10);
	unsorted.push_back(1);
	unsorted.push_back(4);
	unsorted.push_back(60);
	unsorted.push_back(-9);
	unsorted.push_back(-21);
	unsorted.push_back(0);
	
	for (int i = 0; i < (int)unsorted.size(); ++i){
		std::cout << unsorted[i] << " ";
	}
	std::cout << std::endl;

	// Sort
	insertionSort(unsorted.begin(), unsorted.end());

	for (int i = 0; i < (int)unsorted.size(); ++i){
		std::cout << unsorted[i] << " ";
	}
	std::cout << std::endl;
	return 0;
}
