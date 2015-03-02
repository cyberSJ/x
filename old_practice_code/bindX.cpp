#include <stdio.h>
#include <memory>
#include <functional>

/* Lesson learned:
 INSIDE THE SAME CLASS, you can bind another function with this
 function, and provide the argument signature that the "another"
 function requires.

 So bind has 2 syntax:
 1. Normal syntax:
   auto f1 = std::bind(function, arg1, arg2, arg3...);
 
 2. Advanced syntax:
   auto f1 = std::bind(thisClass'sAnotherFunction, this, argOfAnotherFunction);
*/

/* Lesson learned:
 std::ref() passes the reference of the variable to bind() so that 
 the function bound by bind() can use reference as an argument
 instead of another copy of argument supplied when calling bind().

 The founction bound by bind() must accept reference (not copy)
 in order for std::ref() to work properly with bind().

 std::cref() passes the constant reference of the variable and the
 function that accepts the variable cannot modify it.
*/

using namespace std::placeholders;

class MyAdd2{
public:
	int myAdd2(int a, int b, int c){
		return a * b + c;	
	}
};


class MyAdd{
public:
	int myAdd(int a, int b){
		// bind another class's function with this class. 
		// This means calling this class's function is equivalent
		// to calling another class's function.
		// Call another class's function with the arguments
		// that match that another class's signature.
		// Bind binds address of functions.
		auto f1 = std::bind(&MyAdd::myAdd2,this, a, b, 4);
		//auto f1 = std::bind(&MyAdd2::myAdd2,this, a, b, 4);	// <- error becauase you are refereing to another class's method, which is understood as incompatible by the compiler.
		return f1();
	}
	int myAdd2(int a, int b, int c){
		return a * b + c;	
	}
};

void increment(int &x){
	printf("incremeting..\n");
	++x;
}


int main (int argc, char **argv){
	int myInt = 4;
	//auto f1 = std::bind(myAdd2, 3, 5);
	//printf("result: %d\n", f1());
	MyAdd ma;
	printf("result: %d\n", ma.myAdd(3,5));

	int i = 0;
 	std::bind(increment, i);	// creates bind but doesn't call it.	
	printf("i = %d\n", i);

 	std::bind(increment, i)();	// creates bind AND calls it
	printf("i = %d\n", i);

 	std::bind(increment, std::ref(i))();	// creates bind AND calls it
	printf("i = %d\n", i);

 	//std::bind(increment, &i)();				// error because you are passing the address of i, not reference of i; 

	//std::bind(increment, std::cref(i))();	// error because increment() changes the value of i.
	printf("i = %d\n", i);

	increment(i);
	printf("i = %d\n", i);
	
	return 0;
}
