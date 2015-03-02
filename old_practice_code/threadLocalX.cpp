#include <thread>
#include <iostream>

//thread_local int tl;	// need gcc 4.8
//
//void myFunction (int i){
//	tl = i;
//	std::cout << tl << std::endl;
//}

int main (int argc, char **argv){
	tl = 10;
	std::thread t1(myFunction, 1);
	std::thread t1(myFunction, 2);
	std::thread t1(myFunction, 3);
	std::cout << "in main: " << tl << std::endl;
	
	return 0;
}
