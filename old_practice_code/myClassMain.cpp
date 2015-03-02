#include "myClass.h"


int main (int argc, char ** argv){
	char *pchar = "hello";
	MyClass myClass(pchar);
	//printf("%d\n", myClass.print());

	MyClass myClass2;
	//printf("%d\n", myClass2.print());

	printf("%d\n", 1==2 ? 0:-1);
	

	return 0;
}
