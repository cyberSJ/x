#include <functional>
#include <iostream>

int returnInt(int i){
	return i*2;
}

int main (int argc, char **argv){
	std::function<int(int)> ri1 = returnInt;
	std::function<int(int)> ri2 = &returnInt;
	std::cout << ri1(3) << std::endl;
	std::cout << ri2(3) << std::endl;


	return 0;
}
