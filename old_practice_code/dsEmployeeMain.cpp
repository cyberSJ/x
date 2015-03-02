#include <vector>
#include <iostream>
#include "dsEmployee.h"



int main(int argc, char** argv){

	std::vector<Employee> v(3);
	v[0].setValue("George Bush", 400000.00);
	v[1].setValue("Bill Gates", 2000000000.00);
	v[2].setValue("Dr.Phil", 13000000.00);

	std::cout << findMax(v) << std::endl; 
	std::cout << v[0] << std::endl;

	return 0;
}
