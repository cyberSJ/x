#include <vector>
#include <iostream>


template<typename Comparable>
void dsShellSort(std::vector<Comparable> &a){
	for (int gap = a.size()/2; gap > 0; gap /= 2){
	std::cout << "Gap: " << gap;

		for (int i = gap; i < a.size(); i++){
			Comparable tmp = a[i];
			int j = i;

			for ( ; j >= gap && a[j-gap] > tmp; j -= gap){

				a[j] = a[j-gap];
			}
			a[j] = tmp;
		}
	}
	std::cout << std::endl;
}


int main(int argc, char **argv){

	std::vector<int> a;
	a.push_back(10);
	a.push_back(1);
	a.push_back(-3);
	a.push_back(-20);
	a.push_back(50);
	a.push_back(40);
	a.push_back(0);
	a.push_back(-4);
	a.push_back(8);
	a.push_back(99);
	a.push_back(-24);
	a.push_back(-9);

	for(int i = 0; i < a.size(); i++){
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;

	dsShellSort(a);	

	for(int i = 0; i < a.size(); i++){
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;

	return 0;
}
