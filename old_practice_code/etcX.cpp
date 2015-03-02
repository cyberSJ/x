#include <stdio.h>
#include <iostream>
#include <vector>
#include <assert.h>
#include <stdexcept>	// std::invalid_argument
#include <thread>
#include <string>
#include <boost/optional/optional.hpp>
#include <cmath> // fmod()

class testingNull{
public:
	int *pint;
	double *pdouble;
	char *pchar;
public:
	testingNull()
	: pint(NULL),
	  pdouble(nullptr){}
};

template <typename T>
void takesTemplate(T templ){
	std::cout << templ << std::endl;
}

void functionExpectNonconst (char *hello){
	std::cout << "printing: " << hello << std::endl;
}

class OperatorBool{
public:
	OperatorBool(){}
	
	void operator()(){
		printf("operator () called\n");
	}
	
	operator int() const{
		return 4;
	}

	operator bool () const{
		printf("Hello operator bool\n");
		return true;
	}

};

auto returnInt(int i) -> int{
	return i*2;
}

class throwingInConstructor{
	int m_int; 
public:
	throwingInConstructor(int under4) : m_int(under4){
		if (under4 > 4){
			throw std::invalid_argument("over 4");
		}
	}

	void add(int numAdd){
		m_int += numAdd;
		if (m_int > 10) {
			printf("added reuslt over 10\n");
			return;
		}
		printf("Added result: %d\n", m_int);
	}

};

int main (int argc, char** argv){
	auto x = 5;
	printf("x = %d\n", x);

	int *ptr = nullptr;

	enum class Color
	{
		dark,
		light
	};

	enum class Box
	{
		big,
		small
	};

	Color brown = Color::light;
	Box present = Box::small;
	printf("brown = %d\n", brown);
	//if (brown == present)
	//	printf("same\n");
	//else
	//	printf("Not same\n");

	// "Range-based for statement" practice.
	std::vector<int> myVec;
	myVec.resize(10);
	for (auto& x : myVec){
		x = 1;
	}
	
	for (auto x : myVec){
		std::cout << x << std::endl;
	}

	// static_assert practice. Compile-time assertion.
	static_assert(sizeof(int) >= 4, "int needs to be 4 bytes to use this code");
	static_assert(__cplusplus > 199711L, "Program requires C++11 capable compiler");
	printf("Size of int = %ld\n", sizeof(int));
	printf("__cplusplus = %ld\n", __cplusplus);

	// print out absolute file path (including file name)
	// line of the code
	// function that calls this line.
	printf("file: %s\n", __FILE__);
	printf("line: %d\n", __LINE__);
	printf("function: %s\n", __FUNCTION__);

	printf("Assert test...\n");
	//assert(false);	//only true passes...

	try{
		throwingInConstructor thic(5);
		thic.add(4);
	} catch (const std::exception &error){
		std::cout << error.what() << std::endl;
		printf("Couldn't create the constructor\n");
		//return 1;
	}

	printf("I'm here\n");

	printf("concurrent threads : %d\n", std::thread::hardware_concurrency());

	printf("returned int: %d\n", returnInt(2));

	OperatorBool operatorBool;
	if(operatorBool){
		printf("Insdie if loop\n");
	}
	
	if(operatorBool > 3){
		printf("operatorBool is greater than 3\n");
	}
	
	operatorBool();

	/* Lesson learend: const_cast
	 const_cast can cast const object to non-const.
	 Compiler cannot automatically convert from const to 
	 non-const or vice versa.
	*/
	const char *chello = "Hello world";
	//functionExpectNonconst(chello); // error - cannot pass const
	functionExpectNonconst(const_cast<char *>(chello)); 
	char *hello = const_cast<char *>(chello); 
	functionExpectNonconst(hello); 

	takesTemplate<int> (3);

	testingNull tn;
	printf("pint: %p\n", tn.pint);
	printf("pdouble: %p\n", tn.pdouble);
	printf("pchar: %p\n", tn.pchar);

	//double d1 = d2 = d3 = 0.1;
	//printf("doubles: %f\n",d1);

	boost::optional<int> optionalInt;
	//optionalInt = 4;
	if(optionalInt){
		std::cout << "optionalInt set = " << optionalInt.get() << std::endl;
	}
	else{
		//std::cout << "optionalInt not set = " << optionalInt.get() << std::endl;	// can't call get() if boost::optional is not initialized..
		std::cout << "optionalInt not set" << std::endl;	
	}


	// double conditions in for loop
	int var1, var2;
	for (var1 = 0, var2 = -5; var1 < 10 && var2 > -10; var1++, var2--){
		std::cout << "var1 = " << var1 << " var2 = " << var2 << std::endl;
	}

	// For loop condition vs. incrementation... which one is first?
	for(int i = 0; i < 10; i++){
		std::cout << "i = " << i << std::endl;
	}

	std::pair<int, int> p(2,3);
	std::pair<int, int> p2 = p;
	std::pair<int, int> p3;
	//p3 = p + p2;

	std::cout << (370 + 180)%360 - 180 << std::endl;
	std::cout << fmod((179.1 + 180),360) - 180 << std::endl;
	std::cout << 3.2e2 << std::endl;

	return 1;

}
