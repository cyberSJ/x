#include <iostream>
#include "dsVector.h"

int main(int argc, char** argv){
	dsVector<int> myVector(0);
	myVector.push_back(20);
	std::cout << myVector[0] << std::endl;
	myVector.push_back(30);
	std::cout << myVector[1] << std::endl;
	myVector[0] = myVector[1];
	std::cout << myVector[0] << std::endl;

	dsVector<int> myV2(myVector);
	

	for (dsVector<int>::iterator iter = myVector.begin(); iter != myVector.end(); iter++){
		std::cout << "Contents: " << *iter << std::endl;
	}

	for (dsVector<int>::iterator iter = myV2.begin(); iter != myV2.end(); iter++){
		std::cout << "V2 Contnets: " << *iter << std::endl;
	}

	return 0;
}
