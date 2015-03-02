#include <stdio.h>
/* Lesson learned:
 Don't look at the code, it's unrelevant.
 rvalue reference is same as lvalue reference (&) except
 it refers to rvalue (temporary object) as opposed to 
 lvalue (which posseses its own cozy space in memory)

 Used for std::move semantics and "perfect forwarding"
 I don't currently understand "perfect forwarding"
*/

class SomeClass{
public:
	SomeClass(){}

	int returnLvalueFromRvalue


};

int main (int argc, char **argv){
	int lValue = 4;
	int &a = 4;			// error: int &a needs an lvalue. 4 is an rvalue
	int &a = lValue;	// ok

	

	return 0;
}
