#include <iostream>
#include <chrono>
#include <thread>
#include "threadPoolX.h"

int add(int a, int b){
	std::cout << "in add" << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(3));	
	return a+b;
}

int mult(int a, int b){
	std::cout << "in mult" << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));	
	return a*b;
}


int main(int argc, char** argv){
	ThreadPoolX tp(2);
	std::future<int> ret = tp.queueTask(std::move(std::bind(add,3,5)));	
	std::future<int> ret2 = tp.queueTask(std::move(std::bind(mult,3,5)));	
	std::cout << "result mult: " << ret2.get() << std::endl;
	std::cout << "result add: " << ret.get() << std::endl;


	
	return 0;
}
