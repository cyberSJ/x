#include <iostream>
#include <tuple>
#include <vector>

/* Goal: Learn how to construct a variable args + variadic template class

 "..." pack expansion operator used in FUNCTION ARGUMENT must be
 used only for typename or class (not for enum or int)

 Recursive variadic template function call need a base template
 function that is non variadic.

*/

enum Fruit{
	Apple,
	Banana,
	Orange,
	Pear
};

template<Fruit... Fruits>
class FruitBasket
{
	const static int NFRUITS = sizeof...(Fruits);
	int m_int;
public:
	template<typename... Args>
	FruitBasket(char c, int i, Args... args)
	{
		std::cout << "FruitBasket created with " <<
		NFRUITS << " fruits." << std::endl;

		separateTheArgs(args...);

	}

	// Recursively extract the variable function argument.
	template<typename T, typename... Args>
	void separateTheArgs(T arg, Args... args)
	{
		std::cout << "Separated " << arg << std::endl;
		separateTheArgs(args...);
	}

	// Serves as the "base" function
	template<typename T>
	void separateTheArgs(T arg)
	{
		std::cout << "Separated " << arg << std::endl;
	}
};

int main(int argc, char** argv)
{
	FruitBasket<Apple,Banana> fb('c', 4, Apple, Banana);//, Banana, 3);	

	return 0;
}
