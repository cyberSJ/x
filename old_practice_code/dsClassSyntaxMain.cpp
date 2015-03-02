#include "dsClassSyntax.h"
#include <iostream>
#include "dsClassTemplate.h"
#include <vector>
#include "dsEmployee.h"

int main(int arg, char** argv){

	IntCell myIntCell(9);
	//myIntCell = 2;	// Should not compile: type mismatch. "explicit" keyword can help showing error.
	std::cout << "myIntCell initialized: " << myIntCell.read() << std::endl;

	IntCell *secondIC = new IntCell(40);
	std::cout << "secondIC initialized: " << secondIC->read() << std::endl;

	delete secondIC;

	std::vector<std::string> myStrings;
	myStrings.push_back("hello");
	myStrings.push_back("world");
	const std::string &maxFound = myIntCell.findMax(myStrings);	//const means maxFound should not be changed AFTER this line. & means std::cout will NOT copy by value, but call by reference. maxFound is a reference to a string. It is the address of the maxFound...or it is the address of the std::string returned by the myIntCell.findMax(). 
	std::cout << maxFound << std::endl;

	IntCell copyIntCell(myIntCell);
	std::cout << "copyIntCell initialized: " << copyIntCell.read() << std::endl;
	copyIntCell.write(1);

	IntCell operatorIntCell(10);
	operatorIntCell = copyIntCell;
	std::cout << "operatorIntCell equalized: " << operatorIntCell.read() << std::endl;
	
	MemoryCell<float> myMemCell(1.3);
	std::cout << "myMemCell initialized: " << myMemCell.read() << std::endl;

	std::vector<Employee> v(3);
	v[0].setValue("George Bush", 400000.00);
	v[1].setValue("Bill Gates", 2000000000.00);
	v[2].setValue("Dr.Phil", 13000000.00);

	std::cout << findMax(v) << std::endl; 
	//std::cout << v[0] << std::endl;
	

	return 0;
}
