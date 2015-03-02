#include <iostream>
#include <cstring>
#include <set>
#include <functional>



class CaseInsensitiveCompare{

	public:
		bool operator()(const std::string &lhs, const std::string &rhs) const{
			return strcmp(lhs.c_str(), rhs.c_str()) < 0;
		}


};


int main(int argc, char **argv){

	std::set<int> s;
	for (int i = 0; i < 100000; i++)
		s.insert(s.end(),i);

	
	std::set<std::string, CaseInsensitiveCompare> s2;
	//std::set<std::string, std::less<std::string> > s2;
	s2.insert("hello");
	s2.insert("helloe");
	s2.insert("heLLo");
	std::cout << "The size is: " << s2.size() << std::endl;		// Doesn't work since our CaseInsensitiveCompare is not really case insensitive...
	

}
