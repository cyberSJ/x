#include <iostream>

int main (int argc, char **argv){
	int int1 = 3;
	int int2 = 6;
	asm("addl $12, %%eax;"
	    : "=a"(int1)
		: "a"(int1));

	float f1 = 3.1;
	float f2 = 4.1;
	float result;
	//result = f1 * f2;

//	asm("mov 0x0(%%rip), %%eax;"
//	    "move %%eax, -0x1c(%%rbp);"
//		: "=b"(result)
//		: "a"(f1));
	
	//asm("fld %1;"
	//    "fld %2;"
	//	"fstp (%%esp);"
	//	: "=g"(result)
	//	: "g"(f1), "g"(f2));

	std::cout << int1 << ", " << int2 << std::endl;
	std::cout << result << std::endl;


	return 0;
}
