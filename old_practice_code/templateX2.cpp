#include <iostream>
#include <vector>

/* Goal: To understand how variadic template works

 Lessons learned:
 Using "int" as a template means you need to provide actual value for
 int when you instantiate a template class.

 Instantiation of a template class must match the template
 specification.

 In order to track down the flow of the varidiac template code,
 you need to look at the template speicifcation, not the
 "template <blah blah blah>" before the class keyword.

*/

// An enum that can be used as a template argument.
enum Fruit{
	Apple,
	Banana,
	Orange
};

// Arbitrary class
class Arbitrary
{
public:
	Arbitrary()
	{}
};

// THE template
template<typename H, int index, Fruit... Fruits>
class FruitBasket
{
public:
	FruitBasket()
	{
		std::cout << "I'm never gonna get called. "
					 "But I'm needed because I am "
					 "the template signature that "
					 "below specifications depend on.\n";
	}
};


// Template specification that requires at least one Fruit.
// And also the template spcification that separates other fruits
// with the first fruit.
template<int MyNumber, typename T, Fruit Fr, Fruit... Fruits>
class FruitBasket<T, MyNumber, Fr, Fruits...>
	: public FruitBasket<T, MyNumber+1, Fruits...>
{
	using m_parent = FruitBasket<T, MyNumber+1, Fruits...>;
protected:
    using m_parent::m_vector;
	int m_int;
public:
	FruitBasket()
	: m_int(MyNumber)
	{
		std::cout << "FruitBasket with one fruit and other " <<
		sizeof...(Fruits) << " fruits.\n";
		m_vector.push_back(m_int);
	}

	void printVector()
	{
		for (auto i : m_vector)
		{
			std::cout << i << ", ";
		}
		std::cout << std::endl;
	}
};

// Fruit Basket that MUST NOT have any fruit
// This template is used when all the fruits run out.
template<typename Y, int MyNumber>
class FruitBasket<Y, MyNumber>
{
protected:
	std::vector<int> m_vector;
public:
	FruitBasket()
	{
		std::cout << "Starting with no fruits.\n";
	}
};

// Template class with pre-defined template args...just for fun.
template<>
class FruitBasket<int, 4>
{
public:
	FruitBasket()
	{
		std::cout << "FruitBasket with unrelevant template args.\n";
	}
};

// Entry point

int main(int argc, char** argv)
{
	// Calling fruit basket that MUST have at least one fruit.
	// Also calls fruit basket skeleton since we don't have
	// 2nd fruit.
	FruitBasket<Arbitrary, 4, Apple> fb1; 

	// Calling the original template
	std::cout << "--------------------\n";
	FruitBasket<Arbitrary, 4> fbOriginal;
	
	// Calling fruit basket that MUST have at least one fruit.
	// Also calls fruit basket multiple fruits since we have
	// more than 1 fruit.
	std::cout << "--------------------\n";
	FruitBasket<Arbitrary, 4, Apple, Banana, Orange> fb2; 
	fb2.printVector();

	// Calling fruit basket with no template args.
	std::cout << "--------------------\n";
	FruitBasket<int,4> fbNon;

	return 0;
}
