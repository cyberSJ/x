#include "myClass.h"


bool MyClass::print()
{
		if(myConstChar != NULL){
			printf("%s\n", myConstChar);
			return true;
		}
		else{
			printf("NULL memeber string\n");
			return false;
		}
}
