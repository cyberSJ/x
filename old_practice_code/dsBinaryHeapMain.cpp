#include "dsBinaryHeap.h"
#include <iostream>

int main(int argc, char **argv){

	dsBinaryHeap<int> dsbh;
	dsbh.insert(3);
	dsbh.insert(2);
	dsbh.insert(-3);
	dsbh.insert(0);
	std::cout << "Should not be empty(0): " << dsbh.isEmpty() << std::endl;
	int min;
	dsbh.deleteMin(min);
	std::cout << "minimum: " << min << std::endl;
	dsbh.deleteMin(min);
	std::cout << "minimum: " << min << std::endl;
	dsbh.deleteMin(min);
	std::cout << "minimum: " << min << std::endl;
	dsbh.deleteMin(min);
	std::cout << "minimum: " << min << std::endl;
	dsbh.deleteMin(min);
	std::cout << "minimum: " << min << std::endl;
	dsbh.deleteMin(min);
	std::cout << "minimum: " << min << std::endl;
	


	return 0;
}
