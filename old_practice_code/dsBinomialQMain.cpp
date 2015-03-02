#include "dsBinomialQ.h"
#include <iostream>

int main(int argc, char** argv){
	dsBinomialQ<int> dsbq;
	
	dsbq.insert(3);
	dsbq.insert(1);
	dsbq.insert(-10);
	dsbq.insert(22);
	std::cout << "min: " << dsbq.findMin() << std::endl;
	int minItem;
	dsbq.deleteMin(minItem);
	st::cout << "deleted item: " << minItem << std::endl;
	std::cout << "min after deleteMin(): " << dsbq.findMin() << std::endl;

	return 0;
}
