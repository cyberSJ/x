#include "dsBST.h"
#include <iostream>
#include <string>
#include <functional>

int main(int argc, char **argv){

	dsBST<int, std::less<int> > myBST;
	myBST.insert(4);
	myBST.insert(3);
	myBST.insert(10);
	std::cout << "Contains 4?: " << myBST.contains(4) << std::endl;
	std::cout << "Contains 3?: " << myBST.contains(3) << std::endl;
	std::cout << "Contains 10?: " << myBST.contains(10) << std::endl;
	std::cout << "Contains 2?: " << myBST.contains(2) << std::endl;
	myBST.printTree(std::cout);
	std::cout << "Height of the tree: " << myBST.height() << std::endl;
	std::cout << std::endl;

	myBST.remove(3);
	myBST.remove(4);
	myBST.remove(2);
	std::cout << "Contains 4?: " << myBST.contains(4) << std::endl;
	std::cout << "Contains 3?: " << myBST.contains(3) << std::endl;
	std::cout << "Contains 10?: " << myBST.contains(10) << std::endl;
	std::cout << "Contains 2?: " << myBST.contains(2) << std::endl;
	myBST.printTree(std::cout);
	std::cout << "Height of the tree: " << myBST.height() << std::endl;
	std::cout << std::endl;

	return 0;
}
