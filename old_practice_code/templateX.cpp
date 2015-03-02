#include <iostream>

/* Lesson learned:

If you write just a declaration of the template, you can use that
template as THE master template that other specialized template
can base upon.

Template that has JUST declared is not directly instantiable.
It is like abstract template.

Most of the time, Template that uses largest number of variadic is chosen by compiler if there are such possible options to choose.

In case of template SPECIALIZATION,
"typename" keyword just specifies the type of the variable.
It does not define how many number of variables user needs to specify
in order to use the template. The general template is the one that
defines how many number of variables user can use.

Foward-declared template must be implemented either by general template
or a specialized template.

A template variable cannot be unused.

Never forget that variadic argument can have zero size. <-- this lesson is important.

Cannot inherit recursively.

*/


enum Number
{
	zero, one, two, three
};

// Declare THE template. Everything else is just a specialization
// of this master template.
template<typename T1, typename T2, Number... T3>
class twoArg;

// Specialization of the template. 
template<typename T1, typename T2, Number T3, Number... T4>	// The user must NOT use this syntax.
class twoArg<T1, T2, T3, T4...>	// The user must use this syntax since twoArg is DECLARED as taking 2 or more arguments.
: public twoArg<T1, T2, T4...>	// this requires 2-arg only template to be defined because of variadic T4 can be size 0.
{
public:
	typedef twoArg<T1, T2, T4...> parent; // this is same thing as 2-arg template because T4 is assumed to be zero argument.
	using parent::m_x2;
	int m_x;
public:
	twoArg()
	: m_x(4)
	{
		std::cout << m_x << " " << T3 << " and " << m_x2 << std::endl;
	}
};

// The mysterious class that used to give me some thinking process.
template<typename T1, typename T2>
class twoArg<T1, T2>
{
public:
	int m_x2;
public:
	twoArg()
	: m_x2(3)
	{
		std::cout << m_x2 << std::endl;
	}
};

int main(int argc, char** argv)
{
	twoArg<int, double, one, two, one> t1; // 1 is chosen because we are using same type
	//twoArg<int> t1;    // Error. Because we're using only 1 arg.

	//twoArg<int, double, two> t2; //
	//twoArg<int, double, float> t3; // No error because we're using 3 arugment.


	return 0;
}


// TODO:
// Question: How is the parent (which has 2 types + 1 variadic)
// is equal to the specialized template with only 2 types + 0 variadic?
// Possible Answer: Because the parent is vardiac, speicalized template
// (which has only 2 types) is a BASE for the typedef-ed parent
// (which has 2 types + 1 vardiac). I.E, a template with 2 types
// is more general than the template with 2 types and a variadic.
// (This doesn't make sense, but in c++11 it makes sense)
//
// Forward Declare a 2 type + 1 Varidaic template of type C.
// Define template with 4 type names: 2 random types + 1 type C +
// 1 vardiac of type C.
// Insdie that template typedef a parent with the exact types of 
// Forward declareded template.
// Define the forward 
