#ifndef dsInsertionSort_H
#define dsInsertionSort_H

#include <vector>

template<typename Comparable>
void dsInsertionSort(std::vector<Comparable> &a){
	// My way
	// Start with the first element
	// Save the next element to switch
	// if the next element is smaller than current element, then put the next element into current position. Decrease the position looked at --> for loop
	// After the for loop, put the "next element" to the spot.
	// Repeat from the top.
	// Done.

	// My way Implementation
	Comparable nextElement;
	int j;
	for (int p = 1; p < a.size(); p++){
		nextElement = a[p];
		for (j = p; j > 0 && a[j-1] > nextElement; j--){
			a[j] = a[j-1];
		}
		a[j] = nextElement;
	}
}


// Different version which is called by dsQuickSort
template<typename Comparable>
void dsInsertionSort(std::vector<Comparable> &a, int left, int right){

	Comparable nextElement;
	int j;
	for (int p = left+1; p < right+1; p++){
		nextElement = a[p];
		for (j = p; j > left && a[j-1] > nextElement; j--){
			a[j] = a[j-1];
		}
		a[j] = nextElement;
	}

}
#endif
