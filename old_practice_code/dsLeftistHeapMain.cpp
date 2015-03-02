#include "dsLeftistHeap.h"
#include <iostream>

int main(int argc, char **argv){

	dsLeftistHeap<int> dslh;

	dslh.insert(3);
	dslh.insert(5);
	dslh.insert(30);
	dslh.insert(-30);
	int min;
	//dslh.deleteMin(min);
	//std::cout << "the minimum: " << min << std::endl;

	dsLeftistHeap<int> dslhTwo;
	dslhTwo.insert(2);
	dslhTwo.insert(-222);
	dslhTwo.insert(50);
	
	dslh.merge(dslhTwo);
	dslh.deleteMin(min);
	std::cout << "the minimum: " << min << std::endl;
	

	return 0;
}
