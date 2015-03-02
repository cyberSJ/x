#include <iostream>
#include <vector>



template<typename Comparable>
void swap(Comparable &a, Comparable &b);


template<typename Comparable>
void dsHeapSort(std::vector<Comparable> &a);


template<typename Comparable>
void percolateDown(std::vector<Comparable> &a, int i, int n);

inline int leftChild(int i);


template<typename Comparable>
void dsHeapSort(std::vector<Comparable> &a){

	//Build (max)heap by start percolating down the middle element and gradually step left (or less indicies).
	for (int i = a.size()/2; i >= 0; --i){
		percolateDown(a,i,a.size());
	}

	//Sort by deleting the max
	for (int j = a.size()-1; j > 0; j--){
		swap(a[0],a[j]);	// use swap instead of building another deleteMax() routine.
		percolateDown(a,0,j);
	}
}

template<typename Comparable>
void swap(Comparable &a, Comparable &b){
	Comparable tmp = a;
	a = b;
	b = tmp;
}

inline int leftChild(int i){
	return 2 * i + 1;
}

template<typename Comparable>
void percolateDown(std::vector<Comparable> &a, int i, int n){
	Comparable tmp;
	int child;

	for (tmp = a[i]; leftChild(i) < n; i = child){
		child = leftChild(i);
		
		//Choose larger child (larger because we're making a max heap)
		if (child != n - 1 && a[child+1] > a[child]){
			child++;
		}

		//Percolate down if necessary
		if(a[child] > tmp){
			a[i] = a[child];
		} 
		else
			break;
	}

	a[i] = tmp;

}

int main(int argc, char **argv){

	std::vector<int> a;
	a.push_back(-31);
	a.push_back(10);
	a.push_back(1);
	a.push_back(15);
	a.push_back(-3);
	a.push_back(0);
	a.push_back(9);
	a.push_back(-1);
	a.push_back(31);
	a.push_back(11);

	for (int i = 0; i < a.size(); i++){
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;

	dsHeapSort(a);

	for (int i = 0; i < a.size(); i++){
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;

	return 0;
}
