#include "dsHashSC.h"
#include <string>
#include <iostream>

int main(int argc, char **argv){

	dsHashSC<std::string> myHT;
	myHT.insert("Hello");
	std::cout << "Contains Hello?: " << myHT.contains("Hello") << std::endl;
	myHT.insert("Hello");
	if (myHT.insert("Hello")){
		std::cout << "Second copy of Hello was inserted..." << std::endl;
	}
	else{
		std::cout << "Second copy of Hello was NOT inserted." << std::endl;
	}

	bool rm = myHT.remove("course");
	std::cout << "Remove success(1)?: " << rm << std::endl;	
	rm = myHT.remove("Hello");
	std::cout << "Remove success(1)?: " << rm << std::endl;	

	myHT.insert("Yello");
	myHT.insert("Aello");
	myHT.insert("fello");
	myHT.insert("gello");

	std::cout << "Contains(1)?: " << myHT.contains("Yello") << std::endl;

	//Employee myEmp;

	dsHashSC<Employee> myHTEmp;
	std::string bob("Bob");
	Employee myBob(bob, 400.00);
	myHTEmp.insert(myBob);
	std::cout << "Contains bob(1)?: " << myHTEmp.contains(bob) << std::endl;



	//std::vector<int> myV;
	//myV.reserve(10);
	//myV[3] = 2;
	//std::cout << "MyV[3]: " << myV[3] << std::endl;


	return 0;
}
