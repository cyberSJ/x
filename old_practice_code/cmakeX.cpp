#include <stdio.h>
#include <vector>
#include "cmakeConfig.h"
#ifdef USE_MYADD
#include "add.h"
#endif

int main(int argc, char **argv){
	printf("Hello world\n");	
	printf("major version: %d\nminor version: %d\n",
		firstCmake_VERSION_MAJOR,
		firstCmake_VERSION_MINOR);
	printf("My number: %d\n", myNumber);
#ifdef USE_MYADD
	printf("adding 2 + 2 + 2 = %d\n", add(2,2));
#else
	printf("adding 2 + 2 = %d\n", 2 + 2);
#endif

//#ifdef (HAVE_LOG)
//	printf("I have log\n");
//#endif
//
//#ifdef (HAVE_EXP)
//	printf("I have exp\n");
//#endif

	std::vector<int> myVec;
	myVec.resize(10);
	for (auto& x : myVec){
		x = 10;
	}
	
	for (auto x : myVec){
		printf("x = %d\n", x);
	}


	return 0;
}
