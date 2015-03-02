#include <memory>
#include <stdio.h>
/* Lesson learend:
 "shared" means many pointers "own" the same object.
 shared_ptr is strict opposite of unique_ptr.
 Object pointed by many shared_ptr's is not released until
 all the shared_ptr releases the object.
*/

int main (int argc, char **argv){
	std::shared_ptr<int> pint = std::make_shared<int>(40);
	std::shared_ptr<int> pint2 = std::make_shared<int>(50);
	printf("pint2 : %d\n", *pint2);
	pint2 = pint;
	printf("pint2 : %d\n", *pint2);
	printf("pint: %d\n", *pint);	

	// to initialize a shared_ptr member variable:
	// memberVariable(std::make_shared<type>(value)) for primitive types
	// memberVariable(constructorArgument) for constructor that accepts shared_ptr.

	return 0;
}

