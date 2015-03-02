#include <stdio.h>

/* Lesson learned:
 decltype() can used with -> return value syntax.
 "-> decltype(someFunction())"
 is equivalent to:
 "return the type returned by someFunction()
*/

double returnDouble2(int i){
	return (double)i;
}

auto returnDouble(int i) -> decltype(returnDouble2(i)){
	return (int)i;
}

int main (int argc, char **argv){
	
	printf("Returned double: %f\n", returnDouble(3));
	
	return 0;
}
