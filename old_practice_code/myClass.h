#include <stdio.h>
class MyClass{
	const char *myConstChar;

public:
	MyClass(const char *cc = NULL)
		: myConstChar(cc){
		print();
	}
	
	bool print();

};
