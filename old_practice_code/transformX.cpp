#include <cmath>
#include <functional>
#include <algorithm>
#include <iostream>

int main(int argc, char** argv)
{
	std::vector<int> foo;
	std::vector<int> bar;
	std::vector<int> result;

	for (int i = 0; i < 4; ++i)
	{
		foo.emplace_back(i);
	}
	for (int i = 0; i < 4; ++i)
	{
		bar.emplace_back(i+2);
	}
	result.resize(foo.size());

	std::transform(foo.begin(), foo.end(), bar.begin(), result.begin(),
	[](const int& i, const int& j)->int {int imj = i - j; int imj2 = imj*imj; return imj2;});

	for (int i = 0; i < foo.size(); ++i)
	{
		std::cout << result[i] << std::endl;
	}
	return 0;
}
