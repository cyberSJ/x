#include <vector>
#include <iostream>

// Function delcartion
template<typename Comparable>
void dsMergeSort(std::vector<Comparable> &a);

template<typename Comparable>
void dsMergeSort(std::vector<Comparable> &a, std::vector<Comparable> &tmp, int left, int right);

template<typename Comparable>
void dsMerge(std::vector<Comparable> &a, std::vector<Comparable> &tmp, int leftHalfPos , int rightHalfPos, int rightHalfEnd);

template<typename Comparable>
void dsMergeSort(std::vector<Comparable> &a){
	std::vector<Comparable> tmpArray( a.size() );

	dsMergeSort(a, tmpArray, 0, a.size() - 1);		// dsMergeSort(leftArray, rightArray, leftMostIdx, rightMostIdx)
}

template<typename Comparable>
void dsMergeSort(std::vector<Comparable> &a, std::vector<Comparable> &tmp, int left, int right){
	if (left < right){	// if statement exists so that this function return w/o doing anything in the base case.
		int center = (left + right) / 2;
		dsMergeSort(a, tmp, left, center);
		dsMergeSort(a, tmp, center + 1, right);
		dsMerge(a, tmp, left, center + 1, right);
	}
}

template<typename Comparable>
void dsMerge(std::vector<Comparable> &a, std::vector<Comparable> &tmp, int leftHalfPos , int rightHalfPos, int rightHalfEnd){
	// Introduce variables needed for implementation
	int leftHalfEnd = rightHalfPos- 1;
	int tmpIdx = leftHalfPos;
	int numElements = rightHalfEnd - leftHalfPos + 1;
	
	// Push left and right half elements to tmp array in non-decreasing order
	while (leftHalfPos <= leftHalfEnd && rightHalfPos <= rightHalfEnd){
		if (a[leftHalfPos] <= a[rightHalfPos])
			tmp[tmpIdx++] = a[leftHalfPos++];
		else
			tmp[tmpIdx++] = a[rightHalfPos++];
	}

	// Push the remaining leftHalf to tmp, if any
	while (leftHalfPos <= leftHalfEnd){
		tmp[tmpIdx++] = a[leftHalfPos++];
	}

	// Push the remaining leftHalf to tmp, if any
	while (rightHalfPos <= rightHalfEnd){
		tmp[tmpIdx++] = a[rightHalfPos++];
	}

	// Copy tmp back to a
	for (int i = 0; i < numElements; i++, rightHalfEnd--){
		a[rightHalfEnd] = tmp[rightHalfEnd];
	}
}


int main (int argc, char** argv){
	std::vector<int> a;
	a.push_back(10);
	a.push_back(-10);
	a.push_back(34);
	a.push_back(-2);
	a.push_back(0);
	a.push_back(1);
	a.push_back(99);
	a.push_back(-20);
	a.push_back(10);	

	for (int i = 0; i < (int) a.size(); i++){
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;

	dsMergeSort(a);

	for (int i = 0; i < (int) a.size(); i++){
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;
	return 0;
}
