#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <functional>

/*	Implement findMax function
	Comparator object's function takes vector of objects and comparator 
	to conclude comparison.
	It uses compartor's function instead of direct comparision
*/
//template<typename Object, typename Comparator>
//const Object & findMax(const std::vector<Object> arr, Comparator cmp){
//	int maxIndex = 0;
//	for (int i = 0; i < arr.size(); i++){
//		if (cmp.isLessThan(arr[maxIndex], arr[i]))
//			maxIndex = i;
//	}
//
//	return arr[maxIndex];
//}
//
//
///*
//	function object
//	It accepts and compares two strings
//*/
//class CaseInsensitiveCompare{
//	public:
//		bool isLessThan(const std::string &lhs, const std::string &rhs){
//			return strcmp(lhs.c_str(), rhs.c_str()) < 0;
//		}
//};


//////////////////////////////////////////////////////////////////////////////////////////////////////

/*
	Implementing function object with operator() overloading.
	The function object's operator() has been overloaded to take
	two arguments: two strings to compare.
	operator() can take any number of argument.
	Once a function object has been created by operator(),
	it then can use the overloaded operator() to perform things.
	operator() must return bool.
*/
class CaseInsensitiveCompare{
	public:
		bool operator()(const std::string &lhs, const std::string &rhs){
			return strcmp(lhs.c_str(), rhs.c_str()) < 0;	// implementing isLessThan function with operator().
		}
};



/*
	findMax using function object with overloaded operator()
	It takes a vector of objects and function objects as two agruments.
	Then it calls function object's overloaded operator() to compare
	the elements in the vectors
	findMax is a template function
	Function objects are always copied in c++ STD library, so we copy
	by value here, also.
*/
template<typename Object, typename Comparator>
const Object & findMax(const std::vector<Object> &arr, Comparator isLessThan){
	int maxIndex = 0;
	
	for (int i = 0; i < arr.size(); i++){
		if (isLessThan(arr[maxIndex], arr[i]))
			maxIndex = i;
	}

	return arr[maxIndex];
}


//////////////////////////////////////////////////////////////////////////////////////////////////////



/*
	findMax using c++ built-in function object
	It takes just one argument: vector of objects.
	It uses the findMax() that takes two arguments. 
	It uses std::less template function object, which is used as Comparator in the 
	second findMax().
*/
template<typename Object>
const Object & findMax(const std::vector<Object> &arr){
	return findMax(arr, std::less<Object>());
}




/*
	main
	contains vector of strings.
	uses the findMax function and outputs the result to std::cout
*/
int main(int argc, char** argv){
	std::vector<std::string> arr(3);
	arr[0] = "ZERBRA"; arr[1] = "alligator"; arr[2] = "crocodile";
	std::cout << findMax(arr, CaseInsensitiveCompare()) << std::endl;
	std::cout << findMax(arr) << std::endl;

	return 0;
}
