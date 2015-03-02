#include <iostream>
#include "dsInsertionSort.h"

// Function delcaration
template<typename Comparable>
void dsQuickSort(std::vector<Comparable> &a);
template<typename Comparable>
void swap(Comparable &a, Comparable &b);
template<typename Comparable>
void dsQuickSort(std::vector<Comparable> &a, int left, int right);
template<typename Comparable>
Comparable median3(std::vector<Comparable> &a, int left, int right);

// Driver for the real dsQuickSort routine.
template<typename Comparable>
void dsQuickSort(std::vector<Comparable> &a){
	dsQuickSort(a, 0, a.size() - 1);
}

// Main dsQuickSort Routine
template<typename Comparable>
void dsQuickSort(std::vector<Comparable> &a, int left, int right){
	if( left + 10 <= right){
		Comparable pivot = median3(a, left, right);

		// Being partitioing (i.e. seperating smaller and larger)
		int i = left, j = right - 1;
		for (;;){
			while(a[++i] < pivot) {}
			while(a[--j] > pivot) {}		
			if (i < j)
				swap(a[i], a[j]);
			else
				break;	
		}

		// Move back the pivot to pivot position
		swap(a[i], a[right - 1]);

		// Sort the parition with smaller elements
		dsQuickSort(a, left, i-1);
		dsQuickSort(a, i+1, right);
	}
	else
		dsInsertionSort(a, left, right);
}

// Tool used to select the pivot for dsQuickSort
template<typename Comparable>
Comparable median3(std::vector<Comparable> &a, int left, int right){
	int center = (left + right)/2;
	if ( a[center] < a[left] )
		swap(a[center], a[left]);
	if ( a[right] < a[left] )
		swap(a[left], a[right]);
	if ( a[right] < a[center] )
		swap(a[right], a[center]);

	// Place pivot at position right - 1 so it doesn't get in the way of partitioning.
	swap(a[center], a[right-1]);

	return a[right-1];
}

template<typename Comparable>
void swap(Comparable &a, Comparable &b){
	Comparable tmp = a;
	a = b;
	b = tmp;
}


int main(int argc, char **argv){
	std::vector<int> a;
	a.push_back(10);
	a.push_back(0);
	a.push_back(20);
	a.push_back(-4);
	a.push_back(3);
	a.push_back(30);
	a.push_back(-22);
	a.push_back(92);
	a.push_back(-9);
	a.push_back(-9);
	for (int i = 0; i < (int)a.size(); i++){
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;

	dsQuickSort(a);
	for (int i = 0; i < (int)a.size(); i++){
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;
	return 0;
}
