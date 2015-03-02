#include "dsHashProbe.h"
#include <iostream>

int main(int argc, char **argv){

	dsHashProbe<std::string> myHT;

	myHT.insert("Hello");
	std::cout << "Contains (1)?: " << myHT.contains("Hello") << std::endl;
	std::cout << "Contains (1)?: " << myHT.contains("bello") << std::endl;
	
	std::cout << "Wrong remove. should say zero: " << myHT.remove("iello") << std::endl;
	std::cout << "Correct remove. should say one: " << myHT.remove("Hello") << std::endl;

	myHT.insert("Hello");
	std::cout << "Wrong insert. should say zero: " << myHT.insert("Hello") << std::endl;
	myHT.insert("good");
	myHT.insert("world");

	myHT.makeEmpty();
	std::cout << "Contains (1)?: " << myHT.contains("Hello") << std::endl;
	std::cout << "Contains (1)?: " << myHT.contains("good") << std::endl;
	std::cout << "Contains (1)?: " << myHT.contains("world") << std::endl;
	
	

	return 0;
}
