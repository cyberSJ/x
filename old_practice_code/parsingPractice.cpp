#include <stdio.h>
#include <iostream>
#include <stdlib.h>
//#include <string.h>
#include <cstring>
#include <sstream>

int main(int argc, char** argv){

	if (argc != 2) {
		printf("usage: ./parsingPractice word\n");
		exit(1);
	}

	char *myArgument = argv[1];	// save the user input
	printf("I typed %s\n", myArgument);

	char charBox[6]={'a','b','c','d'};
	
	char myChar='a';
	printf("sizeof: %d\n", sizeof(myChar));

	//char command;
	//int cabinetNumber;
	//int successfulRead = sscanf(myArgument,"%s%d",command, cabinetNumber);

	//printf("Read: %d, \t command: %s, \t cabinetNumber: %d\n", successfulRead, command, cabinetNumber);

	char command = myArgument[0];
	int cabinetNumber = myArgument[1];
	//printf("command: %s, \t cabinetNumber: %d\n", myArgument[0], myArgument[1]);
	
	std::cout << myArgument[0] << std::endl;
	std::cout << myArgument[1] << std::endl;

	std::cout << command << std::endl;
	std::cout << cabinetNumber -'0' << std::endl;

	char *emptyTag = "";
	if (emptyTag == NULL || emptyTag == "") printf("tag is empty\n");
	
	std::ostringstream s;
	std::string myString = "Hello";
	std::cout << myString << std::endl;
	int myNumber = 1234;
	s << myString << myNumber;
	std::string newString(s.str());
	std::cout << newString << std::endl;



	return 0;
}
